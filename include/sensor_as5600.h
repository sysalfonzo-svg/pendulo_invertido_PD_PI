#pragma once
#include <Arduino.h>
// Lectura F4 de tres bytes con START repetido. Todos los pasos de bus
// progresan por interrupcion; el planificador impone el plazo por muestra.
namespace captador {
enum Etapa {Libre,Inicio,DireccionW,Registro,Reinicio,DireccionR,Tres,Dos};
static volatile Etapa etapa=Libre;
static volatile bool listo=false,error=false;
static volatile uint32_t fin=0;
static uint8_t bytes[3];
constexpr uint32_t eventos=I2C_CR2_ITEVTEN|I2C_CR2_ITERREN;
inline void iniciarBus() {
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    __HAL_RCC_I2C1_CLK_ENABLE();__HAL_RCC_I2C1_FORCE_RESET();__HAL_RCC_I2C1_RELEASE_RESET();
    const uint32_t reloj=HAL_RCC_GetPCLK1Freq();
    const uint32_t mhz=reloj/1000000u;
    I2C1->CR2=mhz; I2C1->CCR=I2C_CCR_FS | ((reloj+1199999u)/1200000u);
    I2C1->TRISE=(mhz*300u)/1000u+1u;I2C1->OAR1=1u<<14;
    I2C1->CR1=I2C_CR1_PE|I2C_CR1_ACK;
    etapa=Libre;listo=false;error=false;
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn,2,0);HAL_NVIC_SetPriority(I2C1_ER_IRQn,2,0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}
inline bool pedir() {
    if(etapa!=Libre || (I2C1->SR2&I2C_SR2_BUSY)) return false;
    listo=false;error=false;etapa=Inicio;
    I2C1->CR1=I2C_CR1_PE|I2C_CR1_ACK;
    I2C1->CR2|=eventos; I2C1->CR1|=I2C_CR1_START;
    return true;
}
inline void evento() {
    const uint32_t sr=I2C1->SR1;
    switch(etapa) {
    case Inicio:
        if(sr&I2C_SR1_SB) { I2C1->DR=0x36u<<1;etapa=DireccionW; } break;
    case DireccionW:
        if(sr&I2C_SR1_ADDR) { (void)I2C1->SR2;I2C1->DR=0x0b;etapa=Registro; } break;
    case Registro:
        if(sr&I2C_SR1_BTF) {I2C1->CR1|=I2C_CR1_START;etapa=Reinicio;} break;
    case Reinicio:
        if(sr&I2C_SR1_SB) {I2C1->DR=(0x36u<<1)|1u;etapa=DireccionR;} break;
    case DireccionR:
        if(sr&I2C_SR1_ADDR) {(void)I2C1->SR2;etapa=Tres;} break;
    case Tres:
        if(sr&I2C_SR1_BTF) {I2C1->CR1&=~I2C_CR1_ACK;bytes[0]=I2C1->DR;etapa=Dos;} break;
    case Dos:
        if(sr&I2C_SR1_BTF) {
            I2C1->CR1|=I2C_CR1_STOP;bytes[1]=I2C1->DR;bytes[2]=I2C1->DR;
            I2C1->CR2&=~eventos;etapa=Libre;fin=micros();__DMB();listo=true;
        } break;
    default: break;
    }
}
inline void fallo() {
    I2C1->CR2&=~eventos;I2C1->SR1=0;I2C1->CR1|=I2C_CR1_STOP;
    etapa=Libre;error=true;
}
}
extern "C" void I2C1_EV_IRQHandler() {captador::evento();}
extern "C" void I2C1_ER_IRQHandler() {captador::fallo();}
