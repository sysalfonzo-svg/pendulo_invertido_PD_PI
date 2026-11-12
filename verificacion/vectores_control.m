function vectores_control(archivo)
% Reproduce entradas deterministas. El ejecutable prueba_control lee el CSV.
p=parametros_carro();s=zeros(5,1);f=fopen(archivo,'w');assert(f>=0);
cerrar=onCleanup(@()fclose(f));
for k=1:3200
    t=k*p.T;
    escala=[.15,1,3,.5];m=escala(1+floor((k-1)/800));
    y=m*[.03*sin(.7*t); .022*sin(9*t); .19*cos(9*t); .11*sin(5*t); .08*sin(5*t+.4)];
    reiniciar=mod(k-1,800)==0;
    if reiniciar,s=zeros(5,1);end
    [s,u,sat]=paso_control(p,s,y,false);
    fprintf(f,'%.12g,', [reiniciar;y;s(1:3);u]);fprintf(f,'%d\n',sat);
end
end
