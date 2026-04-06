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

```bash
git clone https://github.com/tu-usuario/laboratorio-robot-diferencial.git
cd laboratorio-robot-diferencial
