function resumen=verificar_diseno(graficar)
% Columnas: perturbacion, factor K, delta zona muerta, retardo (muestras),
% factor tau, tiempo final, RMS ultimo segundo [deg], max |x| [m], pico PWM, fallo.
if nargin<1, graficar=true; end
p=disenar_control(false);
casos=[1 1 0 1 1;-1 1 0 1 1;2 1 0 1 1;-2 1 0 1 1;1 .8 3 2 1.2;1 1.15 -4 2 .85;3 1 0 1 1];
resumen=zeros(size(casos,1),10);
for i=1:size(casos,1)
    c=num2cell(casos(i,:)); r=simular_carro(c{:});
    if i==1, nominal=r;end
    resumen(i,:)=[casos(i,:),r.datos(end,1),r.rmsFinal,r.maxX,r.picoPWM,r.fallo];
end
disp('deg   factorK  deltaPWM  atraso  factorTau   tFinal   RMSdeg    maxX_m    PWMpico   fallo');
disp(resumen);
assert(all(resumen(1:4,10)==0) && all(resumen(1:4,7)<0.3),'Falla regresion nominal.');
if graficar
    t=nominal.datos(:,1);d=nominal.datos;
    figure('Name','Equilibrio simulado a 5 ms','Color','w');
    subplot(3,1,1);plot(t,d(:,2),t,0*t,'--');ylabel('Angulo [deg]');legend('Pendulo','Referencia');grid on
    title(sprintf('Perturbacion de 1 grado | radio espectral %.5f',p.rho))
    subplot(3,1,2);plot(t,d(:,7),'k--',t,d(:,5),t,d(:,6));ylabel('Velocidad [m/s]');legend('Referencia','Izquierda','Derecha');grid on
    subplot(3,1,3);stairs(t,d(:,8:9));ylabel('PWM [%]');xlabel('Tiempo [s]');grid on
end
end
