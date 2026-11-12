function [s,u,sat] = paso_control(p,s,y,lineal)
% y=[x relativo; theta; omega filtrada; vIzq; vDer].
% s=[vRef; integral PWM izq; integral PWM der; PWM previo izq; previo der].
% u en % fisico: positivo hacia delante.
a=p.kp*y(2)+p.kd*y(3)+p.kx*y(1)+p.kv*mean(y(4:5));
v=s(1)+p.T*a;
if ~lineal, v=max(-p.vMaxRef,min(p.vMaxRef,v)); end
ae=(v-s(1))/p.T; e=v-y(4:5);
base=(v+p.tau*ae)./p.K + p.kpw.*e;
inc=p.T*p.kiw.*e; trial=base+s(2:3)+inc;
lim=p.pwmMax-p.muerto;
if lineal
    integ=s(2:3)+inc; u=base+integ; sat=false;
else
    ok=abs(trial)<=lim | trial.*inc<=0;
    integ=max(-lim,min(lim,s(2:3)+inc.*ok));
    demand=base+integ; eff=max(-lim,min(lim,demand));
    u=round(eff + p.muerto.*max(-1,min(1,eff/p.suavizado)));
    u(u.*s(4:5)<0)=0; % una muestra sin habilitar antes del cambio de sentido
    sat=any(abs(demand)>=lim) || abs(v)>=p.vMaxRef;
end
s=[v;integ;u];
end
