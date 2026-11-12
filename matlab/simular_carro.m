function r=simular_carro(grados,escalaK,extraMuerto,retardo,escalaTau)
% Ensayo desde una perturbacion con control ya activo. No abre puertos.
if nargin<1, grados=1; end
if nargin<2, escalaK=1; end
if nargin<3, extraMuerto=0; end
if nargin<4, retardo=1; end
if nargin<5, escalaTau=1; end
p=parametros_carro(); T=p.T; duracion=7;
N=round(duracion/T)+1;
assert(retardo>=0 && retardo==floor(retardo));
cola=zeros(2,retardo); z=[0;0;grados*pi/180;0;0;0];
s=zeros(5,1); previo=[]; f=zeros(3,1); cuentaBloqueo=zeros(2,1);
cuentas=2*pi*p.radio./p.cpr; datos=zeros(N,11); falla=0;
for k=1:N
    med=[round(z(3)*4096/(2*pi))*2*pi/4096;round(z(1:2)./cuentas).*cuentas];
    if ~isempty(previo), f=f+T/(p.filtro+T)*((med-previo)/T-f); end
    previo=med; y=[mean(med(2:3));med(1);f];
    inmovil=abs(s(4:5))>=45 & s(4:5).*f(2:3)<.45;
    cuentaBloqueo=(cuentaBloqueo+1).*inmovil;
    if abs(y(2))>25*pi/180 || abs(f(1))>6, falla=8;
    elseif abs(y(1))>.20, falla=16;
    elseif any(abs(f(2:3))>.70), falla=32;
    elseif any(cuentaBloqueo>=30), falla=256;
    end
    if falla==0
        [s,u,sat]=paso_control(p,s,y,false);
    else
        s=zeros(5,1);u=[0;0];sat=false;
    end
    datos(k,:)=[(k-1)*T,z(3)*180/pi,med(1)*180/pi,y(1),f(2:3)',s(1),u',sat,falla];
    if falla~=0 || k==N, break; end
    cola=[cola u]; mando=cola(:,1);cola(:,1)=[];
    ef=sign(mando).*max(0,abs(mando)-p.muerto-extraMuerto);
    objetivo=p.K.*escalaK.*ef; tau=p.tau.*escalaTau;
    h=T;
    k1=deriv(z,objetivo,tau,p); k2=deriv(z+h*k1/2,objetivo,tau,p);
    k3=deriv(z+h*k2/2,objetivo,tau,p); k4=deriv(z+h*k3,objetivo,tau,p);
    z=z+(h/6)*(k1+2*k2+2*k3+k4);
end
r.datos=datos(1:k,:);r.fallo=falla;
r.nombres={'t_s','anguloReal_deg','anguloAS_deg','x_m','vI_m_s','vD_m_s','vref_m_s','PWM_I','PWM_D','saturado','fallo'};
colaFinal=r.datos(:,1)>=max(0,r.datos(end,1)-1);
r.rmsFinal=sqrt(mean(r.datos(colaFinal,2).^2));
r.maxX=max(abs(r.datos(:,4)));r.picoPWM=max(max(abs(r.datos(:,8:9))));
end
function dz=deriv(z,objetivo,tau,p)
dv=(objetivo-z(5:6))./tau;
alpha=p.b*(p.g*sin(z(3))-cos(z(3))*mean(dv));
dz=[z(5:6);z(4);alpha;dv];
end
