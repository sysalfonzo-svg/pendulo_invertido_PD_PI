#include <Arduino.h>
#include <HardwareTimer.h>
#include <stdio.h>
#include <string.h>
#include "state_math.h"
#include "motor_hardware.h"
#include "control.h"
#include "sensor_as5600.h"

#ifndef BANCO
#define BANCO 0
#endif
static_assert(!(EQUILIBRIO && BANCO),"Perfiles mutuamente excluyentes");
#if ENABLE_ESP32_TX && defined(USBCON)
#error "PA11 no puede compartir USB y USART6"
#endif
constexpr uint32_t TS_US=5000, PLAZO_I2C=1500;
static_assert((int)(gan::T*1e6f+0.5f)==TS_US,"Diseno y temporizador deben coincidir");
constexpr uint16_t CERO=2623;
constexpr float RAD=state_math::PI_F/180.0f;
enum Modo : uint8_t { ESPERA, ACTIVO, PARADO, FALLO, PULSO };
enum Causa : uint16_t { SENSOR=1, TIEMPO=2, ENCODER=4, ANGULO=8,
    RECORRIDO=16, VELOCIDAD=32, PARO=64, WATCHDOG=128, RUEDA=256, ENLACE=512 };
volatile Modo modo=ESPERA;
volatile uint16_t fallos=0;
volatile int pwmI=0,pwmD=0;
volatile uint32_t cuentaI=0,cuentaD=0,invalidI=0,invalidD=0,missed=0;
volatile uint32_t inicioPulso=0;
control::Controlador regulador;
IWDG_HandleTypeDef perro={};
uint32_t arranque,ultimoPulso=0;
bool watchdogOk=false;
struct CriticalGuard {
  uint32_t saved;
  CriticalGuard():saved(__get_PRIMASK()) {__disable_irq();__DMB();}
  ~CriticalGuard() {__DMB();__set_PRIMASK(saved);}
};
// HAL tick is 1 kHz, SysTick priority 0. Call ONLY inside CriticalGuard.
// Core micros() does not compensate a SysTick pending while PRIMASK=1.
// All critical sections here are bounded and far shorter than one ms.
uint32_t criticalMicros() {
  uint32_t ms=HAL_GetTick();
  uint32_t ticks=SysTick->VAL;
  const uint32_t reload=SysTick->LOAD+1u;
  if(SCB->ICSR&SCB_ICSR_PENDSTSET_Msk) {++ms;ticks=SysTick->VAL;}
  return ms*1000u+((reload-ticks)*1000u)/reload;
}

void apagar() {pwmI=pwmD=0;motor_hardware::off();}
void parar(uint16_t causa) {
    // Solo con IRQ enmascarada o desde TIM2.
    fallos|=causa;modo=fallos?FALLO:PARADO;apagar();regulador.reset();
}
void aplicar(int izq,int der) {
    pwmI=izq;pwmD=der;
    if(izq||der) motor_hardware::pulse(-izq,-der); // avance fisico = mando de puente negativo
    else motor_hardware::off();
}
const int8_t QUAD[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile uint8_t abI=0,abD=0;
// A se guarda en bit 1, B en bit 0, igual que en la calibracion original.
inline uint8_t par(uint32_t pins,unsigned n) {return ((pins>>n)&1u)*2u+((pins>>(n+1))&1u);}
void flancoI() {uint8_t v=par(GPIOA->IDR,0),old=abI;cuentaI-=QUAD[(old<<2)|v];invalidI+=((old^v)==3);abI=v;}
void flancoD() {uint8_t v=par(GPIOA->IDR,6),old=abD;cuentaD+=QUAD[(old<<2)|v];invalidD+=((old^v)==3);abD=v;}

struct Captura {uint32_t seq,t,iz,der,ei,ed;int pi,pd;};
Captura slot,work;
volatile bool ocupado=false,pendiente=false;
bool adquiriendo=false;
uint32_t sec=0,anteriorTick=0;
HardwareTimer reloj(TIM2);
void tick() {
    const uint32_t t=micros(),dt=t-anteriorTick;
    anteriorTick=t;
    const bool tarde=dt<4900u || dt>5100u;
    if(ocupado || tarde) {
        ++missed;
        if(modo==ACTIVO || modo==PULSO) parar(TIEMPO);
    }
    if(modo==PULSO && t-inicioPulso>=120000u) {apagar();modo=ESPERA;}
    ++sec;
    if(ocupado) return;
    ocupado=true;
    slot={sec,t,cuentaI,cuentaD,invalidI,invalidD,pwmI,pwmD};
    __DMB();pendiente=true;
}
// Cola finita. La adquisicion nunca depende de la capacidad USB.
constexpr size_t TX_SLOTS=8,TX_SIZE=480;
struct TxLine {char data[TX_SIZE];size_t size;};
TxLine tx[TX_SLOTS];
size_t txHead=0,txTail=0,txCount=0,txOffset=0;
uint32_t txDrop=0;
bool streaming=false,greeted=false;
bool enqueue(const char *s) {
#if ENABLE_ESP32_TX
  (void)s;return true; // mensajes de consola descartados sin costo de USB
#else
  size_t n=strlen(s);
  if(txCount==TX_SLOTS||n>=TX_SIZE) return false;
  memcpy(tx[txHead].data,s,n);tx[txHead].size=n;
  txHead=(txHead+1)%TX_SLOTS;++txCount;return true;
#endif
}
void clearTx() {txHead=txTail=txCount=txOffset=0;}
void serviceTx() {
#if ENABLE_ESP32_TX
  return; // no existe puerto USB en el perfil PA11_TX
#else
  // USBSerial::operator bool() in STM32duino 2.11 calls delay(10)!
  // dtr() reads the flag directly, without blocking.
  if(!Serial.dtr()||!txCount) return;
  int freeBytes=Serial.availableForWrite();
  if(freeBytes<=0) return;
  size_t n=tx[txTail].size-txOffset;
  if(n>(size_t)freeBytes) n=freeBytes;
  if(n>64) n=64;
  txOffset+=Serial.write((const uint8_t*)tx[txTail].data+txOffset,n);
  if(txOffset==tx[txTail].size) {txOffset=0;txTail=(txTail+1)%TX_SLOTS;--txCount;}
#endif
}
#if ENABLE_ESP32_TX
// Paquete binario v1, 50 bytes; little endian; IEEE754 float32.
// [0..1] sync A5 5A, [2] version=1, [3] payloadBytes=44
// [4..7] seq, [8..11] t_us, [12..15] encL, [16..19] encR
// [20..21] pwmL, [22..23] pwmR, [24..25] AS5600 raw (FFFF si invalido)
// [26] status AS5600, [27] flags, [28..31] x_m, [32..35] v_m_s
// [36..39] theta_rad, [40..43] omega_rad_s, [44..47] missed
// [48..49] CRC16 CCITT-FALSE sobre bytes [0..47], LSB primero.
// No se espera ACK; se descarta una trama si el DMA no esta disponible.
constexpr uint32_t ESP_BAUD=115200u;
constexpr uint32_t ESP_DIVIDER=10u; // 200Hz control / 10 = 20Hz telemetria
constexpr size_t ESP_PACKET_BYTES=50u;
static_assert(sizeof(float)==4, "Se requiere IEEE754 float32");
struct EspSnapshot {
  uint32_t seq,t_us,encL,encR;
  int16_t pwmL,pwmR;
  uint16_t raw;
  uint8_t status,flags;
  float x,v,theta,omega;
  uint32_t missed;
};
EspSnapshot espSnapshot={};
bool espPending=false,espReady=false;
uint8_t espFrame[ESP_PACKET_BYTES] __attribute__((aligned(4)))={};
uint32_t espDropped=0,espDmaErrors=0;

void espSnapshotSample(int raw,int status,uint8_t flags,
                       float x,float v,float theta,float omega) {
  if((work.seq%ESP_DIVIDER)!=0u) return;
  if(espPending) {++espDropped;return;}
  espSnapshot.seq=work.seq;
  espSnapshot.t_us=work.t;
  espSnapshot.encL=work.der;
  espSnapshot.encR=0u-work.iz;
  espSnapshot.pwmL=(int16_t)-work.pi;
  espSnapshot.pwmR=(int16_t)-work.pd;
  espSnapshot.raw=raw>=0?(uint16_t)raw:0xffffu;
  espSnapshot.status=status>=0?(uint8_t)status:0xffu;
  espSnapshot.flags=flags;
  espSnapshot.x=x;
  espSnapshot.v=v;
  espSnapshot.theta=theta;
  espSnapshot.omega=omega;
  espSnapshot.missed=missed;
  espPending=true;
}

void espPutU16(size_t &p,uint16_t u) {
  espFrame[p++]=(uint8_t)u;espFrame[p++]=(uint8_t)(u>>8);
}
void espPutU32(size_t &p,uint32_t u) {
  for(unsigned i=0;i<4;++i) espFrame[p++]=(uint8_t)(u>>(8*i));
}
void espPutFloat(size_t &p,float f) {
  uint32_t bits;
  memcpy(&bits,&f,sizeof(bits));
  espPutU32(p,bits);
}
uint16_t espCrc16(const uint8_t *data,size_t length) {
  uint16_t crc=0xffffu;
  for(size_t i=0;i<length;++i) {
    crc^=(uint16_t)data[i]<<8;
    for(unsigned j=0;j<8;++j)
      crc=(crc&0x8000u)?(uint16_t)((crc<<1)^0x1021u):(uint16_t)(crc<<1);
  }
  return crc;
}

// Arranque directo de USART6 y DMA, sin HAL_UART, callbacks ni ISR de telemetria.
void initEspTelemetry() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio={};
  gpio.Pin=GPIO_PIN_11;gpio.Mode=GPIO_MODE_AF_PP;
  gpio.Pull=GPIO_PULLUP;gpio.Speed=GPIO_SPEED_FREQ_LOW;
  gpio.Alternate=GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOA,&gpio);

  __HAL_RCC_USART6_CLK_ENABLE();
  USART6->CR1=0u;USART6->CR2=0u;USART6->CR3=0u;
  const uint32_t pclk=HAL_RCC_GetPCLK2Freq();
  USART6->BRR=(pclk+(ESP_BAUD/2u))/ESP_BAUD; // 16x oversampling, 8N1
  USART6->CR3=USART_CR3_DMAT;
  USART6->CR1=USART_CR1_TE|USART_CR1_UE;

  __HAL_RCC_DMA2_CLK_ENABLE();
  DMA2_Stream6->CR&=~DMA_SxCR_EN;
  if(DMA2_Stream6->CR&DMA_SxCR_EN) {espReady=false;return;}
  DMA2_Stream6->CR=0u;
  DMA2_Stream6->FCR=0u; // modo directo, FIFO deshabilitado
  DMA2_Stream6->PAR=(uint32_t)&USART6->DR;
  DMA2_Stream6->M0AR=(uint32_t)espFrame;
  DMA2->HIFCR=DMA_HIFCR_CFEIF6|DMA_HIFCR_CDMEIF6|
              DMA_HIFCR_CTEIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTCIF6;
  // Stream6/Channel5: memoria->periferico, incrementa memoria, prioridad baja.
  DMA2_Stream6->CR=DMA_CHANNEL_5|DMA_SxCR_DIR_0|DMA_SxCR_MINC;
  espReady=true;
}

// Solo en foreground, fuera de la ISR de control y de las secciones criticas.
void serviceEspTelemetry() {
  if(!espReady||!espPending) return;
  if(DMA2_Stream6->CR&DMA_SxCR_EN) return;
  if(!(USART6->SR&USART_SR_TC)) return;
  if(DMA2->HISR&(DMA_HISR_TEIF6|DMA_HISR_DMEIF6|DMA_HISR_FEIF6))
    ++espDmaErrors;
  DMA2->HIFCR=DMA_HIFCR_CFEIF6|DMA_HIFCR_CDMEIF6|
              DMA_HIFCR_CTEIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTCIF6;

  const EspSnapshot &s=espSnapshot;
  espFrame[0]=0xa5;espFrame[1]=0x5a;espFrame[2]=1;espFrame[3]=44;
  size_t p=4;
  espPutU32(p,s.seq);espPutU32(p,s.t_us);
  espPutU32(p,s.encL);espPutU32(p,s.encR);
  espPutU16(p,(uint16_t)s.pwmL);espPutU16(p,(uint16_t)s.pwmR);
  espPutU16(p,s.raw);espFrame[p++]=s.status;espFrame[p++]=s.flags;
  espPutFloat(p,s.x);espPutFloat(p,s.v);
  espPutFloat(p,s.theta);espPutFloat(p,s.omega);
  espPutU32(p,s.missed);
  if(p!=ESP_PACKET_BYTES-2u) {++espDmaErrors;espPending=false;return;}
  espPutU16(p,espCrc16(espFrame,p));
  __DMB();
  DMA2_Stream6->NDTR=(uint16_t)p;
  DMA2_Stream6->M0AR=(uint32_t)espFrame;
  DMA2_Stream6->CR|=DMA_SxCR_EN;
  espPending=false;
}
#endif
constexpr float mI=(float)state_math::METERS_R,mD=(float)state_math::METERS_L;
uint32_t tPrev=0,iPrev=0,dPrev=0,eiPrev=0,edPrev=0,erroresI2C=0;
int64_t posI=0,posD=0;
uint16_t rawPrev=0,buenas=0;
bool primera=true,angPrev=false,datosOk=false;
uint32_t ultimoDato=0;
float x=0,xOrigen=0,vi=0,vd=0,theta=0,omega=0;
unsigned sinRespuesta[2]={};
const char *cabecera="seq,t_us,dt_us,raw,mag,sp_deg,theta_deg,omega_rad_s,x_m,vI_m_s,vD_m_s,vref_m_s,pwmI_pct,pwmD_pct,nI,nD,modo,fallos,sat,missed,i2c,acq_us,tx_drop\n";
void muestra(bool bus,uint32_t edad) {
    const uint32_t dt=primera?TS_US:work.t-tPrev;
    const int32_t di=primera?0:state_math::countDelta(work.iz,iPrev);
    const int32_t dd=primera?0:state_math::countDelta(work.der,dPrev);
    posI+=di;posD+=dd;
    const float ts=dt*1e-6f;
    if(dt) {
        vi=state_math::lowpass(vi,mI*di/ts,ts,gan::filtro);
        vd=state_math::lowpass(vd,mD*dd/ts,ts,gan::filtro);
    }
    x=0.5f*(float)(posI*state_math::METERS_R+posD*state_math::METERS_L);
    const int mag=bus?captador::bytes[0]:-1;
    const int raw=bus?((captador::bytes[1]&15)*256+captador::bytes[2]):-1;
    const bool iman=bus && (mag&0x38)==0x20;
    if(iman) {
        theta=state_math::angle(raw,CERO,-1);
        const float dtheta=-state_math::angleDelta(raw,rawPrev)*state_math::RAD_PER_COUNT;
        omega=angPrev&&dt?state_math::lowpass(omega,dtheta/ts,ts,gan::filtro):0;
        rawPrev=raw;
    } else {theta=NAN;omega=0;}
    uint16_t causa=0;
    if(!iman) causa|=SENSOR;
    if(dt<4900 || dt>5100 || edad>PLAZO_I2C) causa|=TIEMPO;
    if(work.ei!=eiPrev || work.ed!=edPrev) causa|=ENCODER;
    if(fabsf(vi)>.70f || fabsf(vd)>.70f) causa|=VELOCIDAD;
    angPrev=iman;primera=false;tPrev=work.t;iPrev=work.iz;dPrev=work.der;
    eiPrev=work.ei;edPrev=work.ed;ultimoDato=work.t;datosOk=!causa;
    int aplicI,aplicD;Modo estado;uint16_t fault;float ref;bool sat;
    {
        CriticalGuard guard;
        if(modo==PULSO || modo==ACTIVO) {
            if(modo==ACTIVO) {
                if(fabsf(theta)>25*RAD || fabsf(omega)>6) causa|=ANGULO;
                if(fabsf(x-xOrigen)>.20f) causa|=RECORRIDO;
                // Detecta encoder inmovil/reverso ante mando persistente.
                const int u[]={work.pi,work.pd};const float vv[]={vi,vd};
                for(unsigned j=0;j<2;++j) {
                    const bool bloqueada=abs(u[j])>=45 && u[j]*vv[j]<0.45f;
                    sinRespuesta[j]=bloqueada?sinRespuesta[j]+1:0;
                    if(sinRespuesta[j]>=30) causa|=RUEDA;
                }
            }
            if(causa) parar(causa);
        }
        if(EQUILIBRIO && modo==ESPERA) {
            if(work.t-arranque>=10000000u) parar(0);
            else {
                bool vertical=!causa && fabsf(theta)<=1.5f*RAD && fabsf(omega)<.20f &&
                    fabsf(vi)<.025f && fabsf(vd)<.025f;
                buenas=vertical?buenas+1:0;
                if(buenas>=100) {xOrigen=x;regulador.reset();modo=ACTIVO;}
            }
        }
        if(modo==ACTIVO) {
            regulador.paso(x-xOrigen,theta,omega,vi,vd);
            aplicar(regulador.pwm[0],regulador.pwm[1]);
        }
        aplicI=pwmI;aplicD=pwmD;estado=modo;fault=fallos;
        ref=regulador.vRef;sat=regulador.saturado;
    }
#if ENABLE_ESP32_TX
    espSnapshotSample(raw,mag,(iman?0:1)|(fault?8:0),x,0.5f*(vi+vd),theta,omega);
#else
    if(streaming && work.seq%4==0) {
        char r[TX_SIZE];
        snprintf(r,sizeof(r),"%lu,%lu,%lu,%d,%d,0,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%ld,%ld,%u,%u,%d,%lu,%lu,%lu,%lu\n",
            (unsigned long)work.seq,(unsigned long)work.t,(unsigned long)dt,raw,mag,
            (double)(theta/RAD),(double)omega,(double)x,(double)vi,(double)vd,(double)ref,
            aplicI,aplicD,(long)state_math::countDelta(work.iz,0),(long)state_math::countDelta(work.der,0),
            (unsigned)estado,(unsigned)fault,(int)sat,(unsigned long)missed,
            (unsigned long)erroresI2C,(unsigned long)edad,(unsigned long)txDrop);
        if(!enqueue(r)) ++txDrop;
    }
#endif
    if(!bus) ++erroresI2C;
    {
        CriticalGuard guard;
        if(criticalMicros()-work.t>2500u && (modo==ACTIVO || modo==PULSO)) parar(TIEMPO);
        adquiriendo=false;ocupado=false;
    }
    if(watchdogOk) HAL_IWDG_Refresh(&perro);
}
void servicioSensor() {
    if(adquiriendo) {
        if(captador::listo) {
            __DMB();uint32_t edad=captador::fin-work.t;
            bool ok=!captador::error && edad<=PLAZO_I2C;
            if(!ok) captador::iniciarBus();
            muestra(ok,edad);
        } else if(captador::error || micros()-work.t>=PLAZO_I2C) {
            captador::iniciarBus();muestra(false,micros()-work.t);
        }
    } else if(pendiente) {
        __DMB();work=slot;pendiente=false;
        if(micros()-work.t>=PLAZO_I2C || !captador::pedir()) {
            captador::iniciarBus();muestra(false,micros()-work.t);
        } else adquiriendo=true;
    }
}
#if !ENABLE_ESP32_TX
char entrada=0;
bool lineaInvalida=false;
void comando(char c) {
    if(c=='G') {if(enqueue(cabecera)) streaming=true;}
    else if(c=='H') streaming=false;
    else if(c=='!' || c=='S') {CriticalGuard lock;parar(PARO);}
    else if(c=='P' || c=='?') {
        char r[160];snprintf(r,sizeof(r),"# PD_PI perfil=%s modo=%u fallos=%u cero=%u buenas=%u PWM=%d,%d\n",
            EQUILIBRIO?"equilibrio":(BANCO?"banco":"diagnostico"),(unsigned)modo,(unsigned)fallos,
            CERO,buenas,pwmI,pwmD);enqueue(r);
    } else if(BANCO && (c=='I'||c=='D')) {
        CriticalGuard lock;
        const uint32_t now=criticalMicros();
        if(modo==ESPERA && datosOk && now-ultimoDato<10000u && now-ultimoPulso>=1000000u &&
                fabsf(vi)<.02f && fabsf(vd)<.02f) {
            ultimoPulso=inicioPulso=now;modo=PULSO;aplicar(c=='I'?40:0,c=='D'?40:0);
            enqueue("# PULSO 40%, 120 ms\n");
        } else enqueue("# PULSO RECHAZADO: consultar P, inmovilizar y esperar 1 s\n");
    }
}
#endif
void setup() {
    const uint32_t salidas[]={PA8,PA9,PB12,PB13,PB14,PB15};
    for(uint32_t p:salidas) {digitalWrite(p,LOW);pinMode(p,OUTPUT);}
    motor_hardware::begin();
    const uint32_t entradas[]={PA0,PA1,PA6,PA7};
    for(uint32_t p:entradas) pinMode(p,INPUT_PULLUP);
    abI=par(GPIOA->IDR,0);abD=par(GPIOA->IDR,6);
    attachInterrupt(digitalPinToInterrupt(PA0),flancoI,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PA1),flancoI,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PA6),flancoD,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PA7),flancoD,CHANGE);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g={};g.Pin=GPIO_PIN_6|GPIO_PIN_7;g.Mode=GPIO_MODE_AF_OD;
    g.Pull=GPIO_PULLUP;g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;g.Alternate=GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB,&g);captador::iniciarBus();
#if ENABLE_ESP32_TX
    initEspTelemetry();
#else
    Serial.begin(115200);
#endif
    bool reinicioWatchdog=RCC->CSR&RCC_CSR_IWDGRSTF;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    perro.Instance=IWDG;perro.Init.Prescaler=IWDG_PRESCALER_16;perro.Init.Reload=127;
    watchdogOk=HAL_IWDG_Init(&perro)==HAL_OK;
    if(!watchdogOk || reinicioWatchdog) parar(WATCHDOG);
    reloj.pause();reloj.setOverflow(TS_US,MICROSEC_FORMAT);
    reloj.attachInterrupt(tick);reloj.setInterruptPriority(1,0);
    arranque=anteriorTick=micros();reloj.resume();
}
void loop() {
    servicioSensor();
#if ENABLE_ESP32_TX
    serviceEspTelemetry();
#else
    if(!Serial.dtr()) {
        if(modo==PULSO) {CriticalGuard guard;parar(ENLACE);}
        streaming=false;greeted=false;entrada=0;lineaInvalida=false;clearTx();return;
    }
    if(!greeted) {comando('P');greeted=true;}
    for(int j=0;j<16 && Serial.available()>0;++j) {
        const char c=(char)Serial.read();
        if(c=='!') {comando(c);entrada=0;lineaInvalida=true;}
        else if(c=='\r'||c=='\n') {
            if(entrada && !lineaInvalida) comando(entrada);
            entrada=0;lineaInvalida=false;
        } else if(!entrada) entrada=c;
        else lineaInvalida=true;
    }
    serviceTx();
#endif
}
