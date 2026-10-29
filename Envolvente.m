%% Captura de la oscilacion libre del pendulo y calculo de la envolvente
% Cabezal: "INICIO EXPERIMENTO" -> empieza a guardar datos
% Cola:    "FIN"                -> deja de guardar y cierra el puerto
% Formato de datos: 0,Tiempo_ms,Angulo_deg,GyroZ
% Requisitos: MATLAB R2019b o superior.
% Cierra el monitor serial de PlatformIO/Arduino antes de correr esto.

clear; clc;

puerto           = "/dev/ttyUSB0";   % <-- cambia por tu puerto
baudios          = 115200;
timeoutInicio_s  = 40;       % espera maxima del cabezal (el motor corre 5 s al arrancar)
timeoutCaptura_s = 180;      % duracion maxima de la captura
minMuestras      = 200;      % ignora un "FIN" que llegue antes de tener estas muestras
archivoCSV       = "datos_pendulo.csv";

% 4 campos: 0, Tiempo_ms, Angulo (decimal), GyroZ (entero o NaN)
patron = "^\d+,\d+,-?\d+(\.\d+)?,(-?\d+|NaN),?$";

%% Abrir puerto y reiniciar el ESP32
delete(serialportfind);                 % libera puertos abiertos de corridas anteriores
s = serialport(puerto, baudios);
configureTerminator(s, "LF");
s.Timeout = 10;

setDTR(s, false);
setRTS(s, true);
pause(0.1);
setRTS(s, false);
flush(s);

%% 1) Esperar el cabezal
disp("Esperando 'INICIO EXPERIMENTO'...");
t0 = tic;
inicio = false;
while ~inicio && toc(t0) < timeoutInicio_s
    try
        linea = strtrim(readline(s));
    catch
        continue;
    end
    if contains(linea, "INICIO EXPERIMENTO")
        inicio = true;
    end
end
if ~inicio
    clear s
    error("No llego 'INICIO EXPERIMENTO'. Revisa el puerto o presiona EN en el ESP32.");
end

%% 2) Capturar hasta la cola
disp("Capturando...");
filas = zeros(0, 3);                    % [Tiempo_ms, Angulo_deg, GyroZ]
t0 = tic;
fin = false;
while ~fin && toc(t0) < timeoutCaptura_s
    try
        linea = strtrim(readline(s));
    catch
        continue;
    end
    if contains(linea, "FIN") && size(filas, 1) >= minMuestras
        fin = true;
    elseif ~isempty(regexp(linea, patron, "once"))
        v = str2double(split(linea, ","));
        filas(end+1, :) = v(2:4)';      %#ok<SAGROW>
    end
end
clear s                                 % cierra el puerto

if ~fin
    warning("No llego 'FIN'; se usan los datos recibidos hasta ahora.");
end

%% Tabla
T = array2table(filas, "VariableNames", ["Tiempo_ms" "Angulo_deg" "GyroZ"]);
[~, iu] = unique(T.Tiempo_ms, "stable");   % quita tiempos repetidos
T = T(iu, :);
T.Tiempo_s = (T.Tiempo_ms - T.Tiempo_ms(1)) / 1000;
T = movevars(T, "Tiempo_s", "After", "Tiempo_ms");
fprintf("%d muestras en %.1f s (~%.0f Hz)\n", height(T), T.Tiempo_s(end), height(T)/T.Tiempo_s(end));

%% Envolvente
t = T.Tiempo_s;
y = T.Angulo_deg;

prom = 0.01 * max(abs(y));              % prominencia minima para contar un pico
iMax = islocalmax(y, "SamplePoints", t, "MinProminence", prom);
iMin = islocalmin(y, "SamplePoints", t, "MinProminence", prom);
% islocalmax/islocalmin no cuentan el primer punto, que aqui es el pico del
% lanzamiento. Se fuerza como pico si es el extremo de sus primeras muestras.
ini = y(1:min(10, numel(y)));
if y(1) >= max(ini)
    iMax(1) = true;
elseif y(1) <= min(ini)
    iMin(1) = true;
end

assert(nnz(iMax) >= 3 && nnz(iMin) >= 3, "Muy pocos picos; baja la prominencia (prom).");

% Envolvente por picos (interpolada); da NaN fuera del primer/ultimo pico
envSup = interp1(t(iMax), y(iMax), t, "pchip");
envInf = interp1(t(iMin), y(iMin), t, "pchip");

% Envolvente exponencial A0*exp(-sigma*t) ajustada a todos los picos
tp = [t(iMax); t(iMin)];
ap = max(abs([y(iMax); y(iMin)]), eps);
c  = polyfit(tp, log(ap), 1);
sigma  = -c(1);
A0     = exp(c(2));
envExp = A0 * exp(-sigma * t);

Td = mean(diff(t(iMax)));               % periodo amortiguado
fprintf("Envolvente exponencial: A0 = %.3f deg, sigma = %.4f 1/s\n", A0, sigma);
fprintf("Periodo amortiguado: %.4f s (%.3f Hz)\n", Td, 1/Td);

%% Guardar
writetable(T, archivoCSV);
save("datos_pendulo.mat", "T", "envSup", "envInf", "A0", "sigma", "Td");

%% Grafica
figure; hold on; grid on;
plot(t, y, "Color", [0.65 0.65 0.65], "DisplayName", "Angulo");
plot(t, envSup, "r", "LineWidth", 1.5, "DisplayName", "Envolvente (picos)");
plot(t, envInf, "r", "LineWidth", 1.5, "HandleVisibility", "off");
plot(t, envExp, "b--", "LineWidth", 1.2, "DisplayName", "Envolvente exponencial");
plot(t, -envExp, "b--", "LineWidth", 1.2, "HandleVisibility", "off");
plot(t(iMax), y(iMax), "kv", "MarkerSize", 4, "DisplayName", "Picos");
plot(t(iMin), y(iMin), "k^", "MarkerSize", 4, "HandleVisibility", "off");
xlabel("Tiempo [s]"); ylabel("Angulo [deg]");
title("Oscilacion libre del pendulo");
legend("Location", "best");