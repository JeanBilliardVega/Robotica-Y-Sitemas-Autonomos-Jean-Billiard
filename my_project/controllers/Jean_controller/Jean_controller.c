/*
 * Jean_controller.c - Robot diferencial con reinicio de posición
 * Para Webots R2025a
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define DURATION_EXPERIMENT 5000
#define PAUSE_DURATION 2000

#define SPEED_MAX  6.28
#define SPEED_HALF 3.14
#define SPEED_SLOW 2.0

// Posición inicial
const double INITIAL_POSITION[3] = {0.0, 0.0, 0.05};
const double INITIAL_ROTATION[4] = {0.0, 0.0, 1.0, 0.0};

void reset_robot_position(WbNodeRef robot_node, WbFieldRef trans_field, WbFieldRef rot_field) {
    wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_POSITION);
    wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROTATION);
    wb_supervisor_node_reset_physics(robot_node);
    printf("  [OK] Posicion reiniciada\n");
}

void run_experiment(WbDeviceTag left_motor, WbDeviceTag right_motor, 
                    double vl, double vr, int duration_ms) {
    wb_motor_set_velocity(left_motor, vl);
    wb_motor_set_velocity(right_motor, vr);
    
    double start_time = wb_robot_get_time();
    while (wb_robot_step(TIME_STEP) != -1) {
        if (wb_robot_get_time() - start_time >= duration_ms / 1000.0) {
            break;
        }
    }
}

void stop_robot(WbDeviceTag left_motor, WbDeviceTag right_motor) {
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
}

int main(int argc, char **argv) {
    wb_robot_init();
    
    // Motores
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    
    if (left_motor == 0 || right_motor == 0) {
        printf("ERROR: No se encontraron los motores\n");
        return -1;
    }
    
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    
    // Supervisor para reset
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("MI_ROBOT");
    
    if (robot_node == NULL) {
        printf("ERROR: No se encontro DEF MI_ROBOT\n");
        printf("Asegurate que el robot tenga: DEF MI_ROBOT E-puck\n");
        return -1;
    }
    
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    
    printf("\n========================================\n");
    printf("  Robot Diferencial - C\n");
    printf("  Con reinicio automatico\n");
    printf("========================================\n\n");
    
    // Experimentos
    const char* names[] = {"Movimiento Recto", "Curva izquierda", "Rotacion en lugar"};
    double speeds[][2] = {
        {SPEED_HALF, SPEED_HALF},
        {SPEED_SLOW, SPEED_MAX},
        {-SPEED_HALF, SPEED_HALF}
    };
    
    for (int i = 0; i < 3; i++) {
        printf("[Experimento %d] %s\n", i+1, names[i]);
        printf("  vl = %.2f rad/s, vr = %.2f rad/s\n", speeds[i][0], speeds[i][1]);
        
        run_experiment(left_motor, right_motor, speeds[i][0], speeds[i][1], DURATION_EXPERIMENT);
        stop_robot(left_motor, right_motor);
        
        printf("  Reiniciando...\n");
        reset_robot_position(robot_node, trans_field, rot_field);
        
        wb_robot_step(TIME_STEP * 5);
        
        printf("  Pausa...\n\n");
        wb_robot_step(PAUSE_DURATION);
    }
    
    printf("=== EXPERIMENTOS COMPLETADOS ===\n");
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // Mantener robot detenido
    }
    
    wb_robot_cleanup();
    return 0;
}