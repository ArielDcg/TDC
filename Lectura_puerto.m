%% Captura de datos del ESP32 (caracterizacion PWM - pendulo invertido)

clear; clc;

puerto   = "/dev/ttyUSB0";       
baudios  = 115200;
timeout_s = 300;          
archivoCSV = "datos_pwm.csv";

%% Abrir puerto
s = serialport(puerto, baudios);
configureTerminator(s, "LF");
s.Timeout = 2;
limpiar = onCleanup(@() clear("s"));   

setDTR(s, false);
setRTS(s, true);
pause(0.1);
setRTS(s, false);
flush(s);
disp("Esperando datos del ESP32...");

%% Leer y parsear
filas = zeros(0, 4);
t0 = tic;
terminado = false;


patron = "^\d+,\d+,-?\d+(\.\d+)?,(-?\d+|NaN),?$";

while ~terminado && toc(t0) < timeout_s
    try
        linea = strtrim(readline(s));
    catch
        continue;                       % timeout de lectura, seguir esperando
    end
    if ismissing(linea) || linea == ""
        continue;
    end

    if contains(linea, "EXPERIMENTO COMPLETADO")
        terminado = true;
        break;
    end

    if ~isempty(regexp(linea, patron, "once"))
        partes = split(linea, ",");
        vals = str2double(partes(1:4));  % ignora una posible coma final
        filas(end+1, :) = vals(:)';      %#ok<SAGROW>
        if mod(size(filas,1), 10) == 0
            fprintf("  %d muestras (PWM %d%%)\n", size(filas,1), vals(1));
        end
    end
end

if ~terminado
    warning("No llego el mensaje de fin; se guardan los datos recibidos hasta ahora.");
end

%% Tabla
T = array2table(filas, "VariableNames", ...
    ["PWM_pct" "Tiempo_ms" "Angulo_deg" "GyroZ"]);
T.Tiempo_s = (T.Tiempo_ms - T.Tiempo_ms(1)) / 1000;
T = movevars(T, "Tiempo_s", "After", "Tiempo_ms");

disp(head(T));
fprintf("Total: %d muestras, %d niveles de PWM\n", height(T), numel(unique(T.PWM_pct)));

%% Resumen por nivel (con columna para anotar la fuerza medida)
Resumen = groupsummary(T, "PWM_pct", "mean", ["Angulo_deg" "GyroZ"]);
Resumen.Fuerza_N = nan(height(Resumen), 1);   % <-- rellena a mano con tu dinamometro
disp(Resumen);



writetable(T, archivoCSV);
save("datos_pwm.mat", "T", "Resumen");
fprintf("Guardado en %s y datos_pwm.mat\n", archivoCSV);

%% Grafica rapida
figure;
subplot(2,1,1);
plot(T.PWM_pct, T.Angulo_deg); grid on;
ylabel("Angulo [deg]"); title("Respuesta durante la rampa de PWM");
subplot(2,1,2);
plot(T.Tiempo_s, T.GyroZ); grid on;
ylabel("GyroZ [raw]");


%% 
p = polyfit(Resumen.PWM_pct, Resumen.mean_Angulo_deg, 1);   % grado 1 = recta
pendiente = p(1);        % esto sería tu Kuf [N/%]
ordenada  = p(2);

yfit = polyval(p, Resumen.PWM_pct);
%% 
