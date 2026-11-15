
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

str_lin = sprintf('F_{lin} = %.4f \\cdot PWM %+.4f', fit_linear(1), fit_linear(2));
str_exp = sprintf('F_{exp} = %.4fe^{%.4f PWM} + %.4fe^{%.4f PWM}', ...
    fit_exp.a, fit_exp.b, fit_exp.c, fit_exp.d);
texto_ecuaciones = {['\bf{Ecuaciones de Ajuste:}'], str_lin, str_exp};


figure(1)
plot(Resumen.PWM_pct,const*sind(Resumen.mean_Angulo_deg),"LineWidth",3)
hold on
plot(xfit, yfit, "r--", "LineWidth", 2, "DisplayName", "Ajuste lineal")
legend("Datos", "Ajuste lineal", "Location", "best")
grid on
plot(fit_exp)

xlabel("PWM [%]")
ylabel("F[N]")
text(0.05, 0.85, texto_ecuaciones, ...
    'Units', 'normalized', ...
    'Interpreter', 'tex', ...
    'FontSize', 10, ...
    'BackgroundColor', [1 1 1 0.85], ... % Fondo blanco semitransparente
    'EdgeColor', [0.2 0.2 0.2], ...     % Borde gris oscuro
    'Margin', 8);                       % Margen interno del cuadro

legend('Location', 'southeast')
hold off
fprintf("%d * PWM + %d \n",fit_linear(1),fit_linear(2))
fit_exp