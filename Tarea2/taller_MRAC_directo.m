%% Taller MRAC - modelo de referencia y leyes de Lyapunov
% Implementacion DIDACTICA de MRAC DIRECTO para la planta del enunciado.
% El PDF no exige explicitamente MRAC directo ni indirecto: confirmar
% con los apuntes del curso.  Error: e = xm - x.
clear; clc; close all;

%% 1. Datos del enunciado
A = [-0.0175, 0.173,   -9.77,     -5.63;
     -0.192,  -1.09,    -0.846,   64.6;
      0,       0,        0,        1;
      0.0081, -0.0738,   0.0062, -1.9];
B = [-0.428; 4.91; 0; 4.22];
Am = [-0.03,   0.166,   125.6,    37.29;
      -0.052, -1.02,  -1554.7,  -427.82;
       0,       0,       0,        1;
       0.128, -0.0142, -1335.49, -425.12];
Bm = [-138.1; 1584.2; 0; 1361.6];

%% 2. Comprobar estabilidad y condiciones de acoplamiento
fprintf('Autovalores de Am:\n'); disp(eig(Am));
% Ganancias IDEALES aproximadas por minimos cuadrados;
% estas NO se utilizan para adaptar el controlador.
Kideal = pinv(B)*(Am-A);      % 1 x 4, pues u = K*x + kr*r
krideal = pinv(B)*Bm;        % escalar
R_A = Am - (A+B*Kideal);
R_B = Bm - B*krideal;
fprintf('K ideal aproximada [1x4]:\n'); disp(Kideal);
fprintf('Kr ideal aproximada: %.8f\n',krideal);
fprintf('Residuo de Am (Frobenius): %.8g\n',norm(R_A,'fro'));
fprintf('Residuo de Bm (2-norma): %.8g\n',norm(R_B,2));
if norm(R_A,'fro') > 1e-8 || norm(R_B,2) > 1e-8
    fprintf(['AVISO: los datos impresos no satisfacen exactamente ', ...
             'el acoplamiento; las ecuaciones teoricas asumen igualdad exacta.\n']);
end

%% 3. Ecuacion de Lyapunov: Am''*P + P*Am = -Q
Q = 50*eye(4);
P = lyap(Am',Q);              % requiere Control System Toolbox
fprintf('Autovalores de P:\n'); disp(eig(P));
fprintf('Residuo de Lyapunov: %.3e\n',norm(Am'*P+P*Am+Q,'fro'));

%% 4. Modelo de referencia ante un escalon
r0 = 2;                    % amplitud didactica, radianes para theta
rEscalon = @(t) r0;
rPulsos = @(t) r0 * double(mod(t,10)<5); % 5 s ON, 5 s OFF
tramo = [0 60];
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01);
[tref, Xref] = ode15s(@(t,xm) Am*xm+Bm*rEscalon(t), ...
                      tramo,zeros(4,1),opts);
figure('Name','Paso 1: modelo de referencia');
nombres = {'u_x','u_z','theta','q'};
for j=1:4
    subplot(2,2,j);
    plot(tref,Xref(:,j),'LineWidth',1.3); grid on;
    xlabel('Tiempo (s)'); ylabel(nombres{j});
    title(['Referencia: ',nombres{j}]);
end

%% 5. MRAC directo: tasas de adaptacion (ajustables)
GammaX = 50*eye(4);               % simetrica definida positiva
gammaR = 50;                 % positiva
% Estado aumentado z=[x(4); xm(4); Khat_x(4); khat_r(1)]
z0 = zeros(13,1);             % planta, modelo y ganancias en cero
entradas = {rEscalon, rPulsos};
etiquetas = {'Escalon', 'Tren de pulsos'};
uT = cell(1,2);        % Guardar las dos señales de control
tiempos = cell(1,2);  % Guardar los tiempos de cada ensayo


for ensayo=1:2
    r = entradas{ensayo};
    [t,Z] = ode15s(@(t,z) dinamica_mrac(t,z,A,B,Am,Bm,P,...
                           GammaX,gammaR,r),tramo,z0,opts);
    x = Z(:,1:4);
    xm = Z(:,5:8);
    e = xm-x;
    Khat = Z(:,9:12);
    krhat = Z(:,13);
     % Evaluar la referencia en todos los tiempos
    r_valores = arrayfun(r,t);
    uT{ensayo} = sum(Khat.*x, 2) + krhat.*r_valores;
    tiempos{ensayo} = t;
    %% Estados reales y deseados
    figure('Name',['Seguimiento - ',etiquetas{ensayo}]);
    for j=1:4
        subplot(2,2,j);
        plot(t,x(:,j),'b-',t,xm(:,j),'r--','LineWidth',1.1);
        grid on; xlabel('Tiempo (s)'); ylabel(nombres{j});
        title(['Seguimiento de ',nombres{j}]);
        legend('Planta','Modelo','Location','best');
    end
    %% Error de seguimiento
    figure('Name',['Error - ',etiquetas{ensayo}]);
    plot(t,e,'LineWidth',1.1); grid on;
    xlabel('Tiempo (s)'); ylabel('Error e = xm - x');
    legend('e_{u_x}','e_{u_z}','e_{theta}','e_q','Location','best');
    title(['Error de seguimiento - ',etiquetas{ensayo}]);
    %% Evolucion de ganancias
    figure('Name',['Ganancias - ',etiquetas{ensayo}]);
    subplot(2,1,1); plot(t,Khat,'LineWidth',1.1); grid on;
    legend('K_{x1}','K_{x2}','K_{x3}','K_{x4}','Location','best');
    xlabel('Tiempo (s)'); title('Ganancias adaptativas de estado');
    subplot(2,1,2); plot(t,krhat,'LineWidth',1.1); grid on;
    xlabel('Tiempo (s)'); title('Ganancia adaptativa de referencia');

    %% Matriz A+BK al final y comparacion con Am
    Kfinal = Khat(end,:);        % fila 1x4
    krfinal = krhat(end);        % escalar
    Acl_final = A+B*Kfinal;
    fprintf('\n===== %s =====\n',etiquetas{ensayo});
    fprintf('Kx final:\n'); disp(Kfinal);
    fprintf('Kr final: %.8f\n',krfinal);
    fprintf('Norma del error final: %.8g\n',norm(e(end,:)));
    fprintf('A + B*Kx_final =\n'); disp(Acl_final);
    fprintf('Am =\n'); disp(Am);
    fprintf('Norma ||Am - (A+B*K_final)||_F = %.8g\n',...
            norm(Am-Acl_final,'fro'));
end

figure;

subplot(2,1,1)
plot(tiempos{1},uT{1},'b','LineWidth',1.2)
grid on
title('Control u(t) - Escalon')
xlabel('Tiempo (s)')
ylabel('u(t)')

subplot(2,1,2)
plot(tiempos{2},uT{2},'r','LineWidth',1.2)
grid on
title('Control u(t) - Tren de pulsos')
xlabel('Tiempo (s)')
ylabel('u(t)')

%% Funcion local: dinamica aumentada del MRAC DIRECTO
function dz = dinamica_mrac(t,z,A,B,Am,Bm,P,GammaX,gammaR,rfun)
    x = z(1:4);
    xm = z(5:8);
    Kx = z(9:12);
    Kr = z(13);
    r = rfun(t);
    e = xm-x;                    % IMPORTANTE: error del enunciado
    u = Kx.'*x + Kr*r;
    dx = A*x + B*u;
    dxm = Am*xm + Bm*r;
    sigma = e.'*P*B;             % escalar
    dKx = GammaX*x*sigma;        % signo positivo para e=xm-x
    dKr = gammaR*r*sigma;
    dz = [dx; dxm; dKx; dKr];
end
