function [T,archivo]=capturar_placa(puerto,segundos,rueda)
% USB de la STM32. Al terminar (o Ctrl+C) envia paro si es perfil equilibrio.
% En banco, un tercer argumento "I" o "D" solicita UN pulso hacia delante.
if nargin<2, segundos=10; end
if nargin<3, rueda=""; end
rueda=upper(string(rueda));
assert(isscalar(rueda)&&any(rueda==["","I","D"]),'Rueda debe ser I, D o cadena vacia.');
validateattributes(segundos,{'numeric'},{'scalar','finite','>=',2,'<=',600});
s=serialport(puerto,115200,'Timeout',0.25);
configureTerminator(s,"LF");setDTR(s,true);flush(s);
perfil="";cab="";ini=tic;retry=-1;
while toc(ini)<4 && strlength(perfil)==0
    if toc(ini)-retry>.4, writeline(s,"P");retry=toc(ini);end
    if s.NumBytesAvailable==0, pause(.005);continue;end
    txt=leerLinea(s);
    if isempty(txt),continue;end
    token=regexp(char(txt),'^# PD_PI perfil=(diagnostico|banco|equilibrio) ','tokens','once');
    if ~isempty(token),perfil=string(token{1});end
end
assert(strlength(perfil)>0,'No responde el firmware PD_PI por USB. Revise perfil, puerto, DTR y monitor cerrado.');
fin=onCleanup(@()cerrar(s,perfil));
assert(rueda=="" || perfil=="banco",'Un pulso individual requiere banco_usb.');
writeline(s,"G");ini=tic;
while toc(ini)<3 && strlength(cab)==0
    if s.NumBytesAvailable==0,pause(.005);continue;end
    txt=leerLinea(s);
    if ~isempty(txt) && startsWith(txt,"seq,t_us,dt_us,raw,"), cab=txt;end
end
assert(strlength(cab)>0,'Falta la cabecera CSV. Compruebe perfil USB.');
nombres=split(cab,','); n=0; pulso=false; rechazado=false;
mat=zeros(ceil(55*segundos)+25,numel(nombres));ini=tic;
fprintf('Perfil %s. Grabando %.1f s en %s.\n',perfil,segundos,puerto);
while toc(ini)<segundos
    if rueda~="" && ~pulso && toc(ini)>=1 && n>=10
        writeline(s,rueda);pulso=true;
    end
    if s.NumBytesAvailable==0, pause(.002);continue;end
    txt=leerLinea(s);
    if isempty(txt) || strlength(txt)==0,continue;end
    if startsWith(txt,"#"),rechazado=rechazado||contains(txt,"RECHAZADO");continue;end
    valores=sscanf(strrep(char(txt),',',' '),'%f');
    if numel(valores)==numel(nombres), n=n+1;mat(n,:)=valores';end
end
clear fin  % para antes de dibujar o escribir en disco
assert(n>=2,'No se recibieron dos filas completas.');
T=array2table(mat(1:n,:),'VariableNames',cellstr(nombres(:)'));
T.t_s=[0;cumsum(mod(diff(T.t_us),2^32))]*1e-6;
archivo=fullfile(pwd,['carro_' char(perfil) '_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.csv']);
writetable(T,archivo);
fprintf('%d filas. Archivo %s\n',n,archivo);
fprintf('Salto de filas CSV: %d. Incrementos missed/I2C/tx_drop: %g / %g / %g.\n', ...
    sum(mod(diff(T.seq),2^32)~=4),T.missed(end)-T.missed(1),T.i2c(end)-T.i2c(1),T.tx_drop(end)-T.tx_drop(1));
fprintf('Muestras activas: %d de %d; mascara final de fallos: %d.\n',sum(T.modo==1),n,T.fallos(end));
if perfil=="equilibrio" && any(T.modo==1)
    activo=T.modo==1;
    a=T.theta_deg(activo);
    fprintf('En activo: RMS %.3f grados, pico %.3f grados, |theta|<=2 grados en %.1f%% de filas.\n', ...
        sqrt(mean(a.^2)),max(abs(a)),100*mean(abs(a)<=2));
    fprintf('Saturacion %.1f%%; diferencia RMS entre ruedas %.4f m/s.\n', ...
        100*mean(T.sat(activo)),sqrt(mean((T.vI_m_s(activo)-T.vD_m_s(activo)).^2)));
end
if rueda~=""
    if rueda=="I", objetivo=T.nI;otra=T.nD;aplicado=T.pwmI_pct;
    else, objetivo=T.nD;otra=T.nI;aplicado=T.pwmD_pct;end
    idx=find(aplicado>0,1);
    assert(pulso && ~rechazado && ~isempty(idx),'Pulso no observado. Consulte el CSV.');
    desde=max(1,idx-1);
    delta=objetivo(end)-objetivo(desde); cruz=otra(end)-otra(desde);
    fprintf('Encoder solicitado %+.0f cuentas; otro %+.0f.\n',delta,cruz);
    assert(delta>=5 && abs(cruz)<=max(3,0.1*delta),'La rueda no coincide, el signo es incorrecto o se movio la otra rueda. Revise el montaje.');
end
figure('Name','Medicion en la STM32','Color','w');
subplot(3,1,1);plot(T.t_s,[T.theta_deg T.sp_deg]);ylabel('Angulo [deg]');grid on
subplot(3,1,2);plot(T.t_s,[T.vref_m_s T.vI_m_s T.vD_m_s]);ylabel('Velocidad [m/s]');legend('Referencia','Izq','Der');grid on
subplot(3,1,3);stairs(T.t_s,[T.pwmI_pct T.pwmD_pct]);ylabel('PWM [%]');xlabel('Tiempo [s]');grid on
end
function linea=leerLinea(s)
dato=readline(s);
if isempty(dato) || ismissing(string(dato)),linea="";else,linea=strip(string(dato));end
end
function cerrar(s,perfil)
try
    if perfil=="equilibrio", writeline(s,"!");end
    writeline(s,"H");
catch err
    warning('No se confirmo envio de cierre: %s',err.message);
end
end
