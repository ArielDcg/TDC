%% Parametros del modelo (Simulink)

r = 135.0e-3; % radio al motor [m]
lc = 40.67e-3; % al centro de masa [m]
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

s = tf('s');
G = (r/I)/(s^2+s*(B*r^2)/I+(lc*g*m*cos(ThetaLin))/I);
bode(G)
rlocus(G)

