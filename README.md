# Aeropéndulo: modelado, caracterización y control

## Descripción

Este repositorio reúne el diseño, la caracterización experimental, el modelado y el control de un aeropéndulo desarrollado para el curso **Técnicas de Control** de la Universidad Nacional de Colombia, sede Bogotá, durante el periodo 2026-II.

Un motor DC con hélice genera el empuje que mueve un brazo articulado en el plano vertical. Un ESP32 lee la posición angular y la velocidad de giro para registrar la respuesta de la planta e implementar el control de posición.

## Objetivos

- Diseñar y ensamblar la plataforma física del aeropéndulo.
- Caracterizar experimentalmente el empuje del actuador y la fricción de la planta.
- Formular un modelo dinámico y contrastarlo con las mediciones.
- Diseñar e implementar un controlador de posición.

## Estructura del repositorio

| Ruta | Contenido |
|---|---|
| [Caracterizacion/](Caracterizacion/) | Firmware de ensayo, datos, gráficas, scripts MATLAB y modelos Simulink. |
| [CaracterizacionKuf/](Caracterizacion/CaracterizacionKuf/) | Barridos de PWM, ajuste del empuje y comparación dinámica con la planta. |
| [CaracterizacionFriccion/](Caracterizacion/CaracterizacionFriccion/) | Capturas de oscilación libre y ajustes de envolvente. |
| [Control/](Control/) | Modelos Simulink y firmware de control y adquisición. |
| [PID/](Control/PID/) | Controlador PID con referencia escalonada definida en el firmware. |
| [Comparador_PID/](Control/Comparador_PID/) | Controlador PID que recibe una secuencia de referencias por puerto serie. |
| [Respuesta_dinamica/](Control/Respuesta_dinamica/) | Firmware de lazo abierto que aplica una secuencia de PWM y registra la respuesta. |
| [Impresion/](Impresion/) | Modelos STL del ángulo, el inserto del rodamiento y el portaimán. |
| [LaserFinal.dxf](LaserFinal.dxf) | Diseño para corte láser. |
| [Informe/](Informe/) | Informe académico en LaTeX. |

## Plataforma y herramientas

### Hardware

- **Control y adquisición.** ESP32, configurado en PlatformIO como `esp32dev`. La revisión física de la placa queda por confirmar.
- **Actuación.** Motor DC con hélice y puente H. Los modelos del puente H, el motor y la hélice, así como la fuente y los límites eléctricos del montaje, quedan por confirmar.
- **Sensores.** Encoder magnético AS5600 para la posición angular e IMU BMI160 para la velocidad de giro. Los ensayos usan las direcciones I²C `0x36` y `0x68`, respectivamente.
- **Estructura.** Acrílico cortado por láser, piezas impresas en 3D y rodamiento de bolas en el pivote.

| Señal en el firmware | GPIO del ESP32 |
|---|---|
| SDA de los sensores | 21 |
| SCL de los sensores | 22 |
| IN1 del puente H, salida PWM | 25 |
| IN2 del puente H | 26 |

La tabla recoge la asignación del código. El [firmware PID](Control/PID/src/main.cpp) aplica PWM a IN1 y mantiene IN2 en bajo. Antes de energizar la planta, es necesario verificar la alimentación, el cableado, la calibración angular, el recorrido del brazo y el límite de PWM aplicable al montaje.

### Software

- **MATLAB y Simulink.** El repositorio incluye el modelo [PruebasKUF_2023b.slx](Caracterizacion/CaracterizacionKuf/PruebasKUF_2023b.slx). El ajuste de curvas usa Curve Fitting Toolbox y el script de diseño del PID usa Control System Toolbox.
- **PlatformIO con Arduino para ESP32.** Los proyectos declaran sus dependencias en `platformio.ini` y usan comunicación serie a 115200 baudios.
- **LaTeX.** El informe usa la clase IEEEtran y se compila con `pdflatex`.

La versión probada de PlatformIO, el sistema operativo de los ensayos y la compatibilidad de todos los modelos con una misma versión de MATLAB quedan por confirmar. Los scripts de adquisición vienen configurados con `/dev/ttyUSB0`.

## Flujo de caracterización

### Ganancia de empuje respecto al PWM

1. Carga el [firmware de barrido PWM](Caracterizacion/CaracterizacionKuf/Varriaci%C3%B3n_PWM/src/main.cpp). La configuración actual recorre de 0 % a 90 % y vuelve a 0 % en pasos de 2 puntos porcentuales. Espera al menos 3 s por nivel antes de evaluar la estabilidad y exige cinco lecturas estables consecutivas del giróscopo, con 100 ms entre lecturas.
2. Ejecuta [grafica_y_Lectura_puerto.m](Caracterizacion/CaracterizacionKuf/grafica_y_Lectura_puerto.m) para capturar y promediar las lecturas por nivel de PWM.
3. Carga la tabla `Resumen` desde el MAT del ensayo y ejecuta [DespejeKuf.m](Caracterizacion/CaracterizacionKuf/DespejeKuf.m). El script calcula el empuje por equilibrio estático y ajusta una recta y una suma de dos exponenciales.

El [informe](Informe/Secciones/04_definicion_planta.tex) describe un barrido de 10 % a 90 % en pasos de 2 puntos porcentuales y 5 s por nivel. El [script de parámetros](Caracterizacion/Simulik_Caracterizaci%C3%B3n/Parametros_simulink.m) usa una ganancia lineal $K_{uf}=9{,}2568\times10^{-4}$ N por punto porcentual de PWM.

### Fricción

1. Carga el [firmware Friccion](Caracterizacion/CaracterizacionFriccion/Friccion/src/main.cpp) y ejecuta [Envolvente.m](Caracterizacion/CaracterizacionFriccion/Envolvente.m). El firmware actual detecta el inicio de la oscilación libre cerca de `ANGLE_START`, fijado en 150° en el código, mediante el encoder y el giróscopo. El corte del motor al 75 % de PWM descrito en el informe corresponde a otro procedimiento.
2. El script espera hasta 40 s por el inicio del ensayo y captura hasta detectar el reposo o alcanzar 180 s de adquisición.
3. El análisis ajusta envolventes exponenciales a los máximos, mínimos y magnitudes de los picos. Para rehacer una gráfica con los datos guardados de 30°, 60°, 90° o 120°, importa el CSV como tabla `Tk` y ejecuta las secciones **Envolvente** y **Grafica**.

El [script de parámetros](Caracterizacion/Simulik_Caracterizaci%C3%B3n/Parametros_simulink.m) usa $B=0{,}0041$ N·s/m como ajuste para seguir la envolvente. La [comparación dinámica](Caracterizacion/CaracterizacionKuf/ComparacionKUF.m) calcula su propio valor a partir del exponente del ajuste `c`.

## Modelado y control

El modelo rotacional combina el par del empuje, el amortiguamiento viscoso y la gravedad. El informe escribe el par gravitacional como $-mgc\cos\theta$ y linealiza alrededor de un equilibrio con velocidad angular nula. Los scripts de identificación calculan el empuje de equilibrio con el seno del ángulo medido. Al comparar ambos planteamientos, usa el origen angular de cada uno. [Ecuaciones del informe](Informe/Secciones/03_entendimiento_planta.tex) · [Cálculo del empuje](Caracterizacion/CaracterizacionKuf/DespejeKuf.m)

| Parámetro | Valor y procedencia |
|---|---|
| Distancia del pivote al motor | 0,135 m, medida con calibrador. |
| Masa del conjunto móvil | 0,030 kg, medida con balanza. |
| Distancia al centro de masa | Aproximadamente 0,0407 m, estimada en CAD. |
| Inercia respecto al pivote | El informe registra $7{,}94\times10^{-5}$ kg·m². Los scripts usan $3{,}2\cdot382{,}232\times10^{-7}\approx1{,}223\times10^{-4}$ kg·m². |
| Ángulo de linealización del script | 50°, según `ThetaLin` en `Parametros_simulink.m`. |

El [firmware PID](Control/PID/src/main.cpp) usa $K_p=94{,}008$, $K_i=188{,}016$ y $K_d=11{,}751$, con un intervalo nominal de 2 ms medido mediante `millis()` y una salida limitada entre 0 % y 100 % de PWM. La referencia inicia en 130° y aumenta en pasos de 4° cada 4 s hasta 174°, según la condición implementada. El monitor serie muestra la referencia, el ángulo, el error y la salida de control.

### Comparar simulación y planta

1. Carga [Respuesta_dinamica](Control/Respuesta_dinamica/) en el ESP32 y libera el puerto serie.
2. En MATLAB, abre [CaracterizacionKuf/](Caracterizacion/CaracterizacionKuf/), carga `c` desde el MAT de un ensayo de fricción y genera `fit_exp` con `DespejeKuf.m` a partir de un MAT de empuje.
3. Configura el puerto y ejecuta [ComparacionKUF.m](Caracterizacion/CaracterizacionKuf/ComparacionKUF.m). El script simula `PruebasKUF_2023b.slx`, envía la misma secuencia de PWM al ESP32 y compara el ángulo simulado con el medido.

### Cargar el controlador

Abre [Control/PID/](Control/PID/) como proyecto PlatformIO, confirma el entorno `esp32dev`, selecciona el puerto y carga el firmware. El controlador comienza a actuar después de la inicialización. Usa el monitor serie a 115200 baudios para observar el seguimiento.

## Resultados

| Prueba | Configuración | Resultado disponible | Evidencia |
|---|---|---|---|
| Caracterización de empuje | Ensayos ascendentes, descendentes, de ida y vuelta y con niveles aleatorios | Ajustes lineal y exponencial de empuje frente al PWM | [Datos y ajustes](Caracterizacion/CaracterizacionKuf/) |
| Caracterización de fricción | Oscilaciones libres registradas para 30°, 60°, 90° y 120° | Envolventes exponenciales y ajuste $B=0{,}0041$ N·s/m empleado en el script de parámetros | [Datos y gráficas](Caracterizacion/CaracterizacionFriccion/) |
| Comparación dinámica en lazo abierto | Escalones de 10 puntos porcentuales cada 10 s, de 0 % a 80 % y de regreso hasta 10 % | RMSE angular de **2,339°** entre simulación y planta | [Gráfica](Caracterizacion/CaracterizacionKuf/comparacion_dinamica.png) · [CSV](Caracterizacion/CaracterizacionKuf/comparacion_dinamica.csv) |
| Seguimiento en lazo cerrado | PID de posición implementado en el ESP32 | Métricas experimentales pendientes de documentar en el informe | [Firmware](Control/PID/src/main.cpp) · [Sección de resultados](Informe/Secciones/06_resultados.tex) |

## Compilar el informe

Desde la carpeta [Informe/](Informe/), compila [main.tex](Informe/main.tex) dos veces para actualizar las referencias cruzadas.

```sh
pdflatex main.tex
pdflatex main.tex
```

## Equipo

- Samuel David Negrete Lancheros
- Jesús David Sánchez Cobos
- Leonardo Acevedo Monroy
- Ariel Giovanni Cárdenas Santisteban

El informe acredita a los cuatro integrantes como coautores. [Portada del informe](Informe/main.tex)

## Referencias y créditos

- **Curso.** Técnicas de Control, Universidad Nacional de Colombia, sede Bogotá.
- **Docente.** Jorge Iván Sofrony Esmeral.
- **Bibliotecas del firmware.** AS5600 de Rob Tillaart y DFRobot_BMI160, declaradas en la [configuración de PlatformIO](Control/Respuesta_dinamica/platformio.ini).
- **Bibliografía académica.** Pendiente de incorporar al [informe](Informe/main.tex).

## Licencia y uso

La licencia del código, los diseños y la documentación queda por definir. Consulta al equipo antes de reutilizar o redistribuir los contenidos.
