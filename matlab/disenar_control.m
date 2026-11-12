function p = disenar_control(exportar)
% Recalcula, verifica los polos de la cascada discreta y exporta el firmware.
if nargin==0, exportar=true; end
p=parametros_carro();
% Planta continua con una integral auxiliar por rueda para obtener distancias.
% q=[posI,posD,theta,omega,vI,vD]. Entradas: PWM efectivo con signo fisico.
A=zeros(6); B=zeros(6,2); A(1,5)=1; A(2,6)=1; A(3,4)=1;
A(4,3)=p.a; A(4,5:6)=p.b*0.5./p.tau';
A(5:6,5:6)=-diag(1./p.tau); B(5:6,:)=diag(p.K./p.tau);
B(4,:)=-p.b*mean(B(5:6,:),1);
zoh=expm([A B;zeros(2,8)]*p.T);
p.Ad=zoh(1:6,1:6); p.Bd=zoh(1:6,7:8);
% 13 estados: x,theta,omega,vI,vD, tres filtros, vRef, dos integrales,
% dos mandos en transito (retardo conservador de una muestra completa).
% La coordenada lateral integrada no afecta al equilibrio longitudinal.
F=zeros(13);
for j=1:13
    e=zeros(13,1); e(j)=1;
    q=[e(1);e(1);e(2:5)];
    [cs,u]=paso_control(p,[e(9:11);0;0],[e(1:2);e(6:8)],true);
    q1=p.Ad*q+p.Bd*e(12:13);
    deriv=(q1([3 1 2])-q([3 1 2]))/p.T;
    filt=e(6:8)+p.T/(p.T+p.filtro)*(deriv-e(6:8));
    F(:,j)=[mean(q1(1:2));q1(3:6);filt;cs(1:3);u];
end
p.F=F; p.polos=eig(F); p.rho=max(abs(p.polos));
assert(p.rho<1,'El modelo discreto nominal es inestable.');
fprintf('Ts %.1f ms; radio espectral %.8f\n',1e3*p.T,p.rho);
fprintf('PD angulo Kp=%.6f Kd=%.6f; centrado Kx=%.6f Kv=%.6f\n',p.kp,p.kd,p.kx,p.kv);
fprintf('PI izquierda Kp=%.5f Ki=%.5f; derecha Kp=%.5f Ki=%.5f\n',p.kpw(1),p.kiw(1),p.kpw(2),p.kiw(2));
if ~exportar, return; end
dest=fullfile(fileparts(fileparts(mfilename('fullpath'))),'include','ganancias.h');
fid=fopen(dest,'w'); assert(fid>=0,'No se pudo abrir ganancias.h');
limpieza=onCleanup(@()fclose(fid));
fprintf(fid,'#pragma once\n// Generado por matlab/disenar_control.m. Ruedas en orden fisico I/D.\nnamespace gan {\n');
n={'T','kp','kd','kx','kv','vMaxRef','pwmMax','filtro','suavizado'};
for k=1:numel(n), fprintf(fid,'constexpr float %s = %.9ef;\n',n{k},p.(n{k})); end
n={'K','tau','kpw','kiw','muerto'};
for k=1:numel(n), fprintf(fid,'constexpr float %s[2] = {%.9ef, %.9ef};\n',n{k},p.(n{k})); end
fprintf(fid,'}\n');
end
