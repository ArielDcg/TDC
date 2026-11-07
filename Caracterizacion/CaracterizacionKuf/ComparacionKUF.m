%% Comparacion dinamica: simulacion (Simulink) vs planta real (ESP32)
% 1) Simula el modelo con la secuencia de PWM
% 2) Envia la misma secuencia y los mismos tiempos al ESP32 por serial
% 3) Lee la respuesta real y grafica ambas
%
% Protocolo serial (a implementar en el firmware del ESP32):
%   ESP32  -> MATLAB : "ESPERANDO SECUENCIA"                 (al terminar el setup)
%   MATLAB -> ESP32  : "SEQ,<step_ms>,<n>,<p1>,<p2>,...,<pn>" (una sola linea, LF)
%   ESP32  -> MATLAB : "ACK,<step_ms>,<n>"                   (confirma lo recibido)
%   ESP32  -> MATLAB : "INICIO EXPERIMENTO"                  (empieza a aplicar la secuencia)
%   ESP32  -> MATLAB : filas "%PWM,Tiempo_ms,Angulo_deg,GyroZ" (continuo, sin esperar estabilidad)
%   ESP32  -> MATLAB : "EXPERIMENTO COMPLETADO"
%
% Requisitos: MATLAB R2019b o superior, Simulink.
% c (ajuste de la oscilacion) y fit_exp (ajuste de Kuf) deben estar en el workspace.
% Cierra el monitor serial de PlatformIO/Arduino antes de correr esto.

clearvars -except c fit_exp
clc;

assert(exist("c", "var") && exist("fit_exp", "var"), ...
    "Faltan c y fit_exp en el workspace: cargalos antes con load(...).");

%% Parametros del modelo (Simulink)
I = 3.2*382.232e-7; % inercia brazo [kg*m^2]
m = 30.00e-3; % masa  brazo [kg]
g = 9.81; % gravedad [m/s^2]
r = 135.0e-3; % radio al motor [m]
lc = 34.67e-3; % al centro de masa [m]
%========Frición Viscosa =========== 
B = -6*c.b*I/r^2;

%B = 0.0041; %Ajustado para seguir envolvente[Estático]
%========Torque mínimo ==============
Tc = 2.7425e-05;
mu_0 = 0.7; %Frición brazo-Eje experimental (ignorar)
%==============Kuf Lineal===============

% kuf = fit_linear(1);
% kuf_0 = fit_linear(2);

kuf = 9.2568e-04;
kuf_0 = 9.2568e-04;

%==============Exponencial A*exp(b) + C*exp(d)========

%Estos ajustes se pueden sacar de "DespejeKuf.m"

% kuf_a = fit_exp.a;
% kuf_b = fit_exp.b;
% kuf_c = fit_exp.c;
% kuf_d = fit_exp.d;

kuf_a = 0.0642;
kuf_b = 0.0046;
kuf_c = -0.0647;
kuf_d = -0.0187;

%==============Secuecia de entrada =================
repeating_sequence = [0 10 20 30 40 50 60 70 80 70 60 50 40 30 20 10];
step_interval = 10;   % [s] duracion de cada escalon

%=======================================================================
%Este se carga con el código de la carpeta "Respuesta_dinamica" en la esp32
%=======================================================================
%Si quieren correr el simulink conservando los datos tomados de la planta
%para iterar entonces corran la sección "Parametros del modelo (Simulink)"
%luego "1) Simulacion" y por ultimo "Grafica" de este mismo script, eso
%conserva los datos reales y pueden iterar la simulación.
%========================================================================

%% Configuracion
modelo        = "PruebasKUF_2023b";   % <-- nombre del .slx (sin extension)
simEnRadianes = false;                  % true si out.simout trae el angulo en rad
puerto        = "/dev/ttyUSB0"; %cambien el puerto si lo prueban en windows
baudios       = 115200; 
archivoCSV    = "comparacion_dinamica.csv";

% Todos los archivos de salida (.csv, .mat, .png) usan este mismo nombre base
[~, nombreBase] = fileparts(archivoCSV);

step_ms  = round(step_interval * 1000);
nNiveles = numel(repeating_sequence);
tFin     = nNiveles * step_interval;   % duracion total [s]

%% 1) Simulacion
assert(exist(modelo, "file") == 4, "No encuentro el modelo '%s' en el path.", modelo);
out = sim(modelo, "StopTime", num2str(tFin));
sim_out = out.simout;

if isa(sim_out, "timeseries")
    tSim = sim_out.Time;
    ySim = squeeze(sim_out.Data);
elseif isstruct(sim_out)                % formato "Structure with time"
    tSim = sim_out.time;
    ySim = squeeze(sim_out.signals.values);
else                                    % formato "Array": el tiempo esta en out.tout
    tSim = out.tout;
    ySim = sim_out;
end
ySim = ySim(:, 1);                      % si hay varias senales, usa la primera
if simEnRadianes
    ySim = rad2deg(ySim);
end
fprintf("Simulacion lista: %d puntos en %.0f s\n", numel(tSim), tFin);

%% 2) Enviar la secuencia al ESP32
delete(serialportfind);                 % libera puertos abiertos de corridas anteriores
s = serialport(puerto, baudios);
configureTerminator(s, "LF");
s.Timeout = 5;

setDTR(s, false);                       % reinicia el ESP32
setRTS(s, true);
pause(0.1);
setRTS(s, false);
flush(s);

disp("Esperando al ESP32...");
t0 = tic;
listo = false;
while ~listo && toc(t0) < 20
    linea = leerLinea(s);
    if linea == ""
        pause(0.01);
        continue;
    end
    listo = contains(linea, "ESPERANDO SECUENCIA");
end
if ~listo
    clear s
    error("El ESP32 no aviso 'ESPERANDO SECUENCIA'. Revisa el puerto o el firmware.");
end

writeline(s, "SEQ," + step_ms + "," + nNiveles + "," + strjoin(string(repeating_sequence), ","));

esperado = sprintf("ACK,%d,%d", step_ms, nNiveles);
ack = "";
t0 = tic;
while ~startsWith(ack, "ACK") && toc(t0) < 5
    ack = leerLinea(s);
    if ack == ""
        pause(0.01);
    end
end
if ack ~= esperado
    clear s
    error("El ESP32 no confirmo la secuencia (recibi: '%s', esperaba: '%s').", ack, esperado);
end
fprintf("Secuencia enviada: %d niveles de %.1f s (%.0f s en total)\n", nNiveles, step_interval, tFin);

%% 3) Leer la respuesta real
patron    = "^\d+,\d+,-?\d+(\.\d+)?,(-?\d+|NaN),?$";
filas     = zeros(0, 4);
timeout_s = tFin + 60;
grabando  = false;
terminado = false;
t0 = tic;

while ~terminado && toc(t0) < timeout_s
    linea = leerLinea(s);
    if linea == ""
        pause(0.01);
        continue;
    end

    if contains(linea, "INICIO EXPERIMENTO")
        grabando = true;
        disp("Experimento en curso...");
        continue;
    end
    if contains(linea, "EXPERIMENTO COMPLETADO")
        terminado = true;
        break;
    end

    if grabando && ~isempty(regexp(linea, patron, "once"))
        partes = split(linea, ",");
        vals = str2double(partes(1:4));  % ignora una posible coma final
        filas(end+1, :) = vals(:)';      %#ok<SAGROW>
        if mod(size(filas, 1), 50) == 0
            fprintf("  %d muestras (PWM %d%%)\n", size(filas, 1), vals(1));
        end
    end
end
clear s                                 % cierra el puerto

if ~terminado
    warning("No llego 'EXPERIMENTO COMPLETADO'; se usan los datos recibidos hasta ahora.");
end
assert(~isempty(filas), "No llegaron datos de la planta real.");

%% Tabla de la planta real
T = array2table(filas, "VariableNames", ["PWM_pct" "Tiempo_ms" "Angulo_deg" "GyroZ"]);
T.Tiempo_s = (T.Tiempo_ms - T.Tiempo_ms(1)) / 1000;
T = movevars(T, "Tiempo_s", "After", "Tiempo_ms");
fprintf("Real: %d muestras en %.1f s\n", height(T), T.Tiempo_s(end));

%% Error entre simulacion y planta real
[tU, iu] = unique(tSim);                % interp1 exige tiempos sin repetir
ySimEnT = interp1(tU, ySim(iu), T.Tiempo_s, "linear", NaN);
rmse = sqrt(mean((ySimEnT - T.Angulo_deg).^2, "omitnan"));
fprintf("RMSE simulacion vs real: %.3f deg\n", rmse);

%% Grafica
figure;
subplot(2,1,1); hold on; grid on;
plot(T.Tiempo_s, T.Angulo_deg, "Color", [0.65 0.65 0.65], "LineWidth", 1.2, "DisplayName", "Planta real");
plot(tSim, ySim, "r", "LineWidth", 1.5, "DisplayName", "Simulacion");
xlabel("Tiempo [s]"); ylabel("Angulo [deg]");
title(sprintf("Respuesta dinamica: simulacion vs real (RMSE = %.3f deg)", rmse));
legend("Location", "best");

subplot(2,1,2); hold on; grid on;
stairs((0:nNiveles) * step_interval, [repeating_sequence repeating_sequence(end)], ...
       "k", "LineWidth", 1.2, "DisplayName", "PWM enviado");
stairs(T.Tiempo_s, T.PWM_pct, "r--", "DisplayName", "PWM reportado por el ESP32");
xlabel("Tiempo [s]"); ylabel("PWM [%]");
title("Entrada aplicada");
legend("Location", "best");

%% Guardar con el mismo nombre del CSV
writetable(T, archivoCSV);
saveas(gcf, nombreBase + ".png");
save(nombreBase + ".mat", "T", "tSim", "ySim", "repeating_sequence", "step_interval", "rmse");
fprintf("Guardado: %s.csv, %s.png y %s.mat\n", nombreBase, nombreBase, nombreBase);

%% Funciones locales
function linea = leerLinea(s)
    % Devuelve una linea nueva del serial, o "" si no hay nada que leer
    linea = "";
    if s.NumBytesAvailable == 0
        return;
    end
    l = readline(s);
    if ~isempty(l) && ~ismissing(l)
        linea = strtrim(l);
    end
end