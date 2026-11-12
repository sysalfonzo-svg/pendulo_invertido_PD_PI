#pragma once
#include <math.h>
#include "ganancias.h"
namespace control {
inline float recorta(float v,float maximo) {return fmaxf(-maximo,fminf(maximo,v));}
struct Controlador {
    float vRef=0;
    float integral[2]={};
    int pwm[2]={}; // I,D fisicas; positivo = frente del carro
    bool saturado=false;
    void reset() { *this=Controlador{}; }
    void paso(float x,float theta,float omega,float vi,float vd) {
        const float velocidad[2]={vi,vd};
        const float a=gan::kp*theta + gan::kd*omega + gan::kx*x + gan::kv*0.5f*(vi+vd);
        const float vNueva=recorta(vRef+gan::T*a,gan::vMaxRef);
        const float aLimitada=(vNueva-vRef)/gan::T;
        vRef=vNueva;
        saturado=fabsf(vRef)>=gan::vMaxRef;
        for(int i=0;i<2;++i) {
            const float error=vRef-velocidad[i];
            const float base=(vRef+gan::tau[i]*aLimitada)/gan::K[i]+gan::kpw[i]*error;
            const float inc=gan::T*gan::kiw[i]*error;
            const float previsto=base+integral[i]+inc;
            const float limite=gan::pwmMax-gan::muerto[i];
            if(fabsf(previsto)<=limite || previsto*inc<=0)
                integral[i]=recorta(integral[i]+inc,limite);
            const float demanda=base+integral[i];
            const float util=recorta(demanda,limite);
            saturado=saturado || fabsf(demanda)>=limite;
            const int nuevo=(int)lroundf(util+gan::muerto[i]*recorta(util/gan::suavizado,1));
            pwm[i]=(nuevo*pwm[i]<0)?0:nuevo;
        }
    }
};
}
