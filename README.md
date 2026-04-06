# Laboratorio: Robot Diferencial en Webots

## Descripción del laboratorio

Este laboratorio implementa el control cinemático de un **robot diferencial** (e-puck) utilizando el lenguaje **C** en la plataforma de simulación Webots.

### Objetivos

- Identificar y controlar los motores independientes de las ruedas
- Programar diferentes patrones de movimiento variando velocidades
- Observar y documentar las trayectorias resultantes

### Experimentos realizados

| Experimento | Condición | vl (rad/s) | vr (rad/s) | Trayectoria |
|-------------|-----------|------------|------------|-------------|
| 1 | `vr = vl` | 3.14 | 3.14 | Línea recta |
| 2 | `vr > vl` | 2.00 | 6.28 | Curva hacia la izquierda |
| 3 | `vr = -vl` | -3.14 | 3.14 | Rotación en el lugar |

## Requisitos

- **Webots R2025a** o superior
- Compilador C (incluido con Webots)

## Cómo ejecutar la simulación

### 1. Clonar el repositorio

git clone [https://github.com/JeanBilliardVega/Robotica-Y-Sitemas-Autonomos-Jean-Billiard.git]
cd 

### 2. Abrir el mundo en Webots
webots worlds/laboratorio.wbt
O desde la interfaz: File → Open World

### 3. Compilar el controlador
En Webots: Tools → Compile controllers o presiona Ctrl+Shift+B

### 4. Ejecutar la simulación
Presiona el botón Run (▶) en Webots

### Resultados obtenidos
Experimento 1: Movimiento Recto (vr = vl)
Ambas ruedas giran a la misma velocidad. El robot avanza en línea recta sin desviación.

Observación: La trayectoria es perfectamente rectilínea. El centro instantáneo de rotación (ICR) se encuentra en el infinito.

Experimento 2: Trayectoria Curva (vr > vl)
La rueda derecha gira más rápido que la izquierda, generando un giro hacia la izquierda. El robot describe un arco de circunferencia.

Observación: El radio de curvatura es constante mientras las velocidades se mantienen fijas.

Experimento 3: Rotación en el lugar (vr = -vl)
Las ruedas giran a igual velocidad pero en sentido contrario. El robot rota sobre su propio eje central sin desplazarse linealmente.

Observación: El centro instantáneo de rotación coincide exactamente con el centro geométrico del robot.

Las pruebas del experimento exitoso se encuentran en la carpeta: [Pruebas en Video del Funcionamiento del Robot]
