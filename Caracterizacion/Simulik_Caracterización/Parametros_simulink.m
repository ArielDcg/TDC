%% Parametros del modelo (Simulink)

r = 135.0e-3; % radio al motor [m]
lc = 38.67e-3; % al centro de masa [m]
I = 3.2*382.232e-7; % inercia brazo [kg*m^2]
m = 30.00e-3; % masa  brazo [kg]
g = 9.81; % gravedad [m/s^2]
ThetaLin = deg2rad(50);
%========Frición Viscosa =========== 
%B = -c.b*I/r^2;

B = 0.0041; %Ajustado para seguir envolvente[Estático]
%B = 0.0495; %Ajustado para mejor respuesta dinámica

%========Torque mínimo ==============[Experimentos]
Tc = 2.7425e-05;
mu_0 = 0.7; %Frición brazo-Eje experimental (ignorar)
%==============Kuf Lineal===============

% kuf = fit_linear(1);
% kuf_0 = fit_linear(2);

kuf = 9.2568e-04;
kuf_0 = 9.2568e-04;

%==============Exponencial A*exp(b) + C*exp(d)========

% kuf_a = fit_exp.a;
% kuf_b = fit_exp.b;
% kuf_c = fit_exp.c;
% kuf_d = fit_exp.d;

kuf_a = 0.0642;
kuf_b = 0.0046; 
kuf_c = -0.0647;
kuf_d = -0.0187;

%==============Secuecia de entrada =================
repeating_sequence = [0 2 4 6 8];
step_interval = 10;   % [s] duracion de cada escalon
%%
s = tf('s');
Kuf_exp = (kuf_a)/(s+kuf_b)+(kuf_c)/(s+kuf_d)

G = kuf*(r/I)/(s^2+s*(B*r^2)/I+(lc*g*m*cos(ThetaLin))/I);
G_exp = Kuf_exp*(r/I)/(s^2+s*(B*r^2)/I+(lc*g*m*cos(ThetaLin))/I);

%bode(G)
rlocus(G)
kp = 94.008;

% Ganancia integral
rlocus(-G/(s*(-1-kp*G)))
% hold on

ki = 188.016;
CPI = (kp+ki/s);
zpk(feedback(CPI*G,1))
step(feedback(CPI*G,1))

rlocus(s*G/(1+(ki/s)*G+kp*G))
kd = 11.751;
CPID = (kp+ki/s+s*kd);
zpk(feedback(CPID*G,1))
step(feedback(CPID*G,1));
Sys = CPID*G;
%controlSystemDesigner(G)

% %% Red combinada
% %Red de adelanto
% bode(Sys)
% mfrc=85;
% wgc= 8.84; %visual, se puede iterar
% [mm ff]=bode(Sys,wgc);
% phiComb=mfrc - (ff+180) + 6;
% bcomb= (1+sin(phiComb*pi/180))/(1-sin(phiComb*pi/180))
% T2Comb = 1/(sqrt(bcomb)*wgc)
% RadComb = (1+bcomb*T2Comb*s)/(1+T2Comb*s);
% Sys2 = Sys * RadComb;
% 
% [NumLead, DenLead] = tfdata(RadComb, 'v');
