#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define GRID_SIZE 8

// --- CONFIGURACIÓN DEL ROBOT ---
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define CELL_SIZE 0.25
#define SAFETY_DISTANCE 0.20

// --- FACTOR DE CORRECCIÓN DE GIRO (ajústalo empíricamente) ---
#define TURN_CORRECTION 1.2   // 1.00 = teórico, >1 gira más, <1 gira menos

// --- MAPA CORREGIDO ---
int grid[GRID_SIZE][GRID_SIZE] = {
    {0, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {0, 0, 0, 1, 1, 0, 1, 1},
    {1, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0}
};

int start_x = 3, start_y = 3;
int goal_x = 7, goal_y = 0;

// --- ESTRUCTURAS A* ---
typedef struct {
    int x, y, g, h, f;
    int parent_x, parent_y;
    bool closed, open;
} Node;

Node map_nodes[GRID_SIZE][GRID_SIZE];
int path_x[GRID_SIZE * GRID_SIZE];
int path_y[GRID_SIZE * GRID_SIZE];
int path_length = 0;

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void calculate_a_star() {
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            map_nodes[i][j].x = i; map_nodes[i][j].y = j;
            map_nodes[i][j].g = 9999; map_nodes[i][j].f = 9999;
            map_nodes[i][j].closed = false; map_nodes[i][j].open = false;
        }
    }
    map_nodes[start_x][start_y].g = 0;
    map_nodes[start_x][start_y].h = heuristic(start_x, start_y, goal_x, goal_y);
    map_nodes[start_x][start_y].f = map_nodes[start_x][start_y].h;
    map_nodes[start_x][start_y].open = true;

    while (1) {
        int current_x = -1, current_y = -1;
        int lowest_f = 99999;
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (map_nodes[i][j].open && map_nodes[i][j].f < lowest_f) {
                    lowest_f = map_nodes[i][j].f;
                    current_x = i; current_y = j;
                }
            }
        }
        if (current_x == -1 || (current_x == goal_x && current_y == goal_y)) break;
        map_nodes[current_x][current_y].open = false;
        map_nodes[current_x][current_y].closed = true;
        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};
        for (int i = 0; i < 4; i++) {
            int nx = current_x + dx[i];
            int ny = current_y + dy[i];
            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && grid[nx][ny] == 0) {
                if (map_nodes[nx][ny].closed) continue;
                int tentative_g = map_nodes[current_x][current_y].g + 1;
                if (!map_nodes[nx][ny].open) map_nodes[nx][ny].open = true;
                else if (tentative_g >= map_nodes[nx][ny].g) continue;
                map_nodes[nx][ny].parent_x = current_x;
                map_nodes[nx][ny].parent_y = current_y;
                map_nodes[nx][ny].g = tentative_g;
                map_nodes[nx][ny].h = heuristic(nx, ny, goal_x, goal_y);
                map_nodes[nx][ny].f = map_nodes[nx][ny].g + map_nodes[nx][ny].h;
            }
        }
    }
    int cx = goal_x, cy = goal_y;
    int temp_x[GRID_SIZE * GRID_SIZE], temp_y[GRID_SIZE * GRID_SIZE];
    int count = 0;
    while (cx != start_x || cy != start_y) {
        temp_x[count] = cx; temp_y[count] = cy;
        int px = map_nodes[cx][cy].parent_x;
        int py = map_nodes[cx][cy].parent_y;
        cx = px; cy = py;
        count++;
    }
    for (int i = 0; i < count; i++) {
        path_x[i] = temp_x[count - 1 - i];
        path_y[i] = temp_y[count - 1 - i];
    }
    path_length = count;
}

void print_robot_map(int current_x, int current_y, int heading) {
    printf("\n====================================\n");
    printf("POSICIÓN ACTUAL: X=%d, Y=%d\n", current_x, current_y);
    printf("ORIENTACIÓN: %d\n", heading);
    printf("--- MAPA ---\n");
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (i == current_x && j == current_y) printf("[R]");
            else if (i == goal_x && j == goal_y) printf("[G]");
            else if (grid[i][j] == 1) printf("███");
            else printf(" . ");
        }
        printf("\n");
    }
    printf("====================================\n\n");
}

double ir_to_meters(double raw) {
    if (raw <= 20) return 0.5;
    double dist = 0.05 * (4096.0 / raw - 1.0);
    if (dist > 0.4) dist = 0.4;
    if (dist < 0.02) dist = 0.02;
    return dist;
}

typedef struct { double d_est, P, Q, R; } Kalman;
void kalman_init(Kalman *k, double d0) { k->d_est = d0; k->P = 1.0; k->Q = 0.001; k->R = 0.05; }
void kalman_predict(Kalman *k, double ds) { k->d_est -= ds; k->P += k->Q; }
void kalman_update(Kalman *k, double z) {
    double K = k->P / (k->P + k->R);
    k->d_est += K * (z - k->d_est);
    k->P *= (1.0 - K);
}

void set_velocity_mode(WbDeviceTag motor) { wb_motor_set_position(motor, INFINITY); }
void set_position_mode(WbDeviceTag motor, double target) { wb_motor_set_position(motor, target); }

int main(int argc, char **argv) {
    wb_robot_init();

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    WbDeviceTag left_ps = wb_robot_get_device("left wheel sensor");
    WbDeviceTag right_ps = wb_robot_get_device("right wheel sensor");
    WbDeviceTag front_right = wb_robot_get_device("ps0");
    WbDeviceTag front_left  = wb_robot_get_device("ps7");
    WbDeviceTag right_sensor = wb_robot_get_device("ps2");
    WbDeviceTag left_sensor  = wb_robot_get_device("ps5");

    wb_position_sensor_enable(left_ps, TIME_STEP);
    wb_position_sensor_enable(right_ps, TIME_STEP);
    wb_distance_sensor_enable(front_right, TIME_STEP);
    wb_distance_sensor_enable(front_left, TIME_STEP);
    wb_distance_sensor_enable(right_sensor, TIME_STEP);
    wb_distance_sensor_enable(left_sensor, TIME_STEP);

    // --- CORRECCIÓN DEL GIRO: se multiplica por TURN_CORRECTION ---
    double turn_90_rad = (M_PI / 2.0) * (AXLE_LENGTH / 2.0) / WHEEL_RADIUS * TURN_CORRECTION;
    double forward_cell_rad = CELL_SIZE / WHEEL_RADIUS;

    int path_index = 0;
    int current_heading = 0;

    typedef enum { GET_NEXT_NODE, TURNING, FORWARD, EVASION, DONE } State;
    State state = GET_NEXT_NODE;
    State previous_state = GET_NEXT_NODE;

    double target_left_pos = 0.0, target_right_pos = 0.0;
    bool target_set = false;

    calculate_a_star();
    printf(">>> INICIANDO SISTEMA A* CON CORRECCIÓN DE GIRO (factor %.2f) <<<\n", TURN_CORRECTION);
    printf("RUTA CALCULADA (%d pasos):\n", path_length);
    for (int i = 0; i < path_length; i++) {
        printf("Paso %d -> (%d,%d)\n", i+1, path_x[i], path_y[i]);
    }
    print_robot_map(start_x, start_y, current_heading);

    wb_robot_step(TIME_STEP);
    double init_l = wb_position_sensor_get_value(left_ps);
    double init_r = wb_position_sensor_get_value(right_ps);
    set_position_mode(left_motor, init_l);
    set_position_mode(right_motor, init_r);
    wb_motor_set_velocity(left_motor, MAX_SPEED * 0.5);
    wb_motor_set_velocity(right_motor, MAX_SPEED * 0.5);

    double prev_l_pos = init_l, prev_r_pos = init_r;

    Kalman kalman;
    kalman_init(&kalman, 0.5);

    bool evasion_active = false;
    int evade_phase = 0;
    double retreat_start_avg = 0.0;
    double retreat_distance = 0.15;
    const int TURN_CYCLES = (int)(25 * TURN_CORRECTION);  // también corregido
    int turn_counter = 0;
    int turn_dir = 1;

    while (wb_robot_step(TIME_STEP) != -1) {
        double l_pos = wb_position_sensor_get_value(left_ps);
        double r_pos = wb_position_sensor_get_value(right_ps);
        double fr_raw = wb_distance_sensor_get_value(front_right);
        double fl_raw = wb_distance_sensor_get_value(front_left);
        double lat_r_raw = wb_distance_sensor_get_value(right_sensor);
        double lat_l_raw = wb_distance_sensor_get_value(left_sensor);

        double front_dist = (ir_to_meters(fr_raw) + ir_to_meters(fl_raw)) / 2.0;
        double left_dist  = ir_to_meters(lat_l_raw);
        double right_dist = ir_to_meters(lat_r_raw);

        double delta_l = (l_pos - prev_l_pos) * WHEEL_RADIUS;
        double delta_r = (r_pos - prev_r_pos) * WHEEL_RADIUS;
        double delta_s = (delta_l + delta_r) / 2.0;
        prev_l_pos = l_pos;
        prev_r_pos = r_pos;

        kalman_predict(&kalman, delta_s);
        if (fr_raw > 20 || fl_raw > 20) kalman_update(&kalman, front_dist);
        double d_est = kalman.d_est;

        static int log_cnt = 0;
        if (++log_cnt % 8 == 0) {
            printf("[KALMAN] d_est=%.3f m | state=%d\n", d_est, state);
        }

        // Detectar obstáculo
        if (state != EVASION && state != DONE && d_est < SAFETY_DISTANCE) {
            printf("[EVASION] Obstáculo a %.3f m.\n", d_est);
            previous_state = state;
            state = EVASION;
            evasion_active = false;
        }

        // --- EVASIÓN (con giro corregido) ---
        if (state == EVASION) {
            if (!evasion_active) {
                set_velocity_mode(left_motor);
                set_velocity_mode(right_motor);
                evasion_active = true;
                evade_phase = 0;
                retreat_start_avg = (l_pos + r_pos) / 2.0;
                turn_dir = (left_dist > right_dist) ? -1 : 1;
                printf("[EVASION] Retrocediendo %.2f m...\n", retreat_distance);
            }

            if (evade_phase == 0) {
                double current_avg = (l_pos + r_pos) / 2.0;
                double retreated = (retreat_start_avg - current_avg) * WHEEL_RADIUS;
                if (retreated < retreat_distance) {
                    wb_motor_set_velocity(left_motor, -MAX_SPEED * 0.4);
                    wb_motor_set_velocity(right_motor, -MAX_SPEED * 0.4);
                } else {
                    evade_phase = 1;
                    turn_counter = TURN_CYCLES;
                    printf("[EVASION] Retroceso completado. Girando...\n");
                }
            } else if (evade_phase == 1) {
                if (turn_counter > 0) {
                    wb_motor_set_velocity(left_motor,  turn_dir * MAX_SPEED * 0.5);
                    wb_motor_set_velocity(right_motor, -turn_dir * MAX_SPEED * 0.5);
                    turn_counter--;
                } else {
                    double current_l = wb_position_sensor_get_value(left_ps);
                    double current_r = wb_position_sensor_get_value(right_ps);
                    set_position_mode(left_motor, current_l);
                    set_position_mode(right_motor, current_r);
                    wb_motor_set_velocity(left_motor, MAX_SPEED * 0.5);
                    wb_motor_set_velocity(right_motor, MAX_SPEED * 0.5);
                    state = previous_state;
                    target_set = false;
                    evasion_active = false;
                    printf("[EVASION] Evasión completada.\n");
                }
            }
            continue;
        }

        // --- NAVEGACIÓN A* (GIRO CORREGIDO) ---
        if (state == DONE) {
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            continue;
        }

        if (state == GET_NEXT_NODE) {
            if (path_index >= path_length) {
                state = DONE;
                printf("\n>>> META ALCANZADA <<<\n");
                continue;
            }
            int next_x = path_x[path_index];
            int next_y = path_y[path_index];
            int dx = next_x - start_x;
            int dy = next_y - start_y;
            int target_heading = current_heading;
            if (dx == -1) target_heading = 0;
            else if (dy == 1) target_heading = 1;
            else if (dx == 1) target_heading = 2;
            else if (dy == -1) target_heading = 3;

            int turn_diff = target_heading - current_heading;
            if (turn_diff < -2) turn_diff += 4;
            if (turn_diff > 2) turn_diff -= 4;

            printf("[LOG] Nodo %d (%d,%d) -> ", path_index, next_x, next_y);
            if (turn_diff != 0) {
                printf("GIRAR\n");
                target_left_pos = l_pos + (turn_diff * turn_90_rad);
                target_right_pos = r_pos - (turn_diff * turn_90_rad);
                state = TURNING;
            } else {
                printf("AVANZAR\n");
                target_left_pos = l_pos + forward_cell_rad;
                target_right_pos = r_pos + forward_cell_rad;
                state = FORWARD;
            }
            target_set = false;
        }

        if (state == TURNING || state == FORWARD) {
            if (!target_set) {
                wb_motor_set_position(left_motor, target_left_pos);
                wb_motor_set_position(right_motor, target_right_pos);
                target_set = true;
            }
            double error_l = fabs(target_left_pos - l_pos);
            double error_r = fabs(target_right_pos - r_pos);
            if (error_l < 0.05 && error_r < 0.05) {
                if (state == TURNING) {
                    int turn_diff = (target_left_pos > l_pos) ? 1 : -1;
                    if (fabs(target_left_pos - l_pos) > turn_90_rad * 1.5) turn_diff *= 2;
                    current_heading = (current_heading + turn_diff + 4) % 4;
                    printf("[LOG] Giro completado. Nueva orientación: %d\n", current_heading);
                    target_left_pos = l_pos + forward_cell_rad;
                    target_right_pos = r_pos + forward_cell_rad;
                    state = FORWARD;
                    target_set = false;
                } else if (state == FORWARD) {
                    start_x = path_x[path_index];
                    start_y = path_y[path_index];
                    path_index++;
                    state = GET_NEXT_NODE;
                    print_robot_map(start_x, start_y, current_heading);
                }
            }
        }
    }

    wb_robot_cleanup();
    return 0;
}