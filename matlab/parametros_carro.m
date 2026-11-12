function p = parametros_carro()
% Magnitudes en SI. Vectores de rueda siempre [izquierda; derecha] fisicas.
p.T = 0.005;
p.m = 0.0273; p.l = 0.265; p.Jp = 0.00253432; p.g = 9.81;
p.b = p.m*p.l/p.Jp; p.a = p.b*p.g;
p.radio = 0.0325; p.cpr = [1974;1969.6];
p.K = [1.3355;1.1062]*(2*pi*p.radio/60); % (m/s) por punto porcentual PWM
p.tau = [0.1688;0.1573];                 % identificacion al aire
p.muerto = [25;30]; p.pwmMax = 70; p.vMaxRef = 0.25;
p.filtro = 0.012; p.lambda = 0.055; p.suavizado = 2;
% theta'' = a*theta-b*x'', a_cmd=kp*theta+kd*omega+kx*x+kv*v.
% Coeficientes de s^4+(b*kd-kv)s^3+(b*kp-kx-a)s^2+a*kv*s+a*kx.
c = poly([-5 -5.5 -1.4 -1.6]);
p.kx=c(5)/p.a; p.kv=c(4)/p.a;
p.kp=(c(3)+p.kx+p.a)/p.b; p.kd=(c(2)+p.kv)/p.b;
% PI de velocidad por cancelacion nominal de primer orden.
p.kpw = p.tau./(p.K*p.lambda); p.kiw = 1./(p.K*p.lambda);
end
