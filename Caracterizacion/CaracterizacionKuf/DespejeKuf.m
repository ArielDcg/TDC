
r = 135e-3; % radio al motor [m]
lc = 40.677e-3; % al centro de masa [m]
m = 30e-3; % masa  brazo [kg]
g = 9.81; % gravedad [m/s^2]

const = m*g*lc/r;

% Cargar el .mat da los datos de experiemtno de angulo - pwm,hay 4
% experimentos en la carpeta CaracterizacionKuf (ida_y_vuelta, ascendente,
% descendente, aleatorio)

%========================Parámetros para simulink ================
fit_linear = polyfit(Resumen.PWM_pct,const*sind(Resumen.mean_Angulo_deg),1);
fit_exp = fit(Resumen.PWM_pct,const*sind(Resumen.mean_Angulo_deg),"exp2");
% ========================================================================
xfit = linspace(min(Resumen.PWM_pct), max(Resumen.PWM_pct), 100);
yfit = polyval(fit_linear, xfit);


figure(1)
plot(Resumen.PWM_pct,const*sind(Resumen.mean_Angulo_deg),"LineWidth",3)
hold on
plot(xfit, yfit, "r--", "LineWidth", 2, "DisplayName", "Ajuste lineal")
legend("Datos", "Ajuste lineal", "Location", "best")
grid on
plot(fit_exp)

xlabel("PWM [%]")
ylabel("F[N]")
hold off
fprintf("%d * PWM + %d \n",fit_linear(1),fit_linear(2))
fit_exp