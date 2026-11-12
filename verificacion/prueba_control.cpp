#include "control.h"
#include <cstdio>
#include <cmath>
// Entrada: salida de vectores_control.m; comparacion del nucleo entregado.
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    FILE *in=std::fopen(argv[1],"r");
    if(!in) return 2;
    control::Controlador c;
    int fila=0,reset,pi,pd,sat;
    double x,th,w,vi,vd,vref,ii,id;
    while(std::fscanf(in,"%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d",
        &reset,&x,&th,&w,&vi,&vd,&vref,&ii,&id,&pi,&pd,&sat)==12) {
        if(reset)c.reset();
        c.paso(x,th,w,vi,vd);++fila;
        if(c.pwm[0]!=pi || c.pwm[1]!=pd || c.saturado!=bool(sat) ||
           std::abs(c.vRef-vref)>2e-6 || std::abs(c.integral[0]-ii)>.002 ||
           std::abs(c.integral[1]-id)>.002) {
            std::fprintf(stderr,"Diferencia en muestra %d: PWM %d/%d vs %d/%d\n",fila,c.pwm[0],c.pwm[1],pi,pd);return 1;
        }
    }
    if(fila!=3200 || !std::feof(in)) return 1;
    std::printf("%d muestras coincidentes en PWM y saturacion; estados dentro de tolerancia.\n",fila);
    std::fclose(in);
}
