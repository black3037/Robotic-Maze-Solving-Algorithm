// Includes
#include "globs.h"
#include "physical_constants.h"
#include "main_control_task.h"
#include "leds_task.h"
#include "debug_printf.h"
#include "robot_settings.h"
#include "glob_types.h"

#define QTR_THRESHOLD (0.5f)
#define NOMINAL_SPEED (0.35f)

enum maze_mode_t
{
    TRACK_LINE,
    TURN_LEFT,
    TURN_RIGHT,
    ADVANCE_LEFT,
    ADVANCE_RIGHT,
    TERMINATION,
    STEADY_STATE_LEFT,
    STEADY_STATE_RIGHT,
    RIGHT_CHECK,
    NODE,
    RETURN_HOME
};

// Global Functions
float determine_line_pos(uint8_t qtr_state, float last_line_pos, uint8_t num_qtr_on);

// Global Tuning
// ORIGIAL PID CONTROLLER (NOMINAL SPEED 0.25f)
//                                 kp       ki      kd
//PidController track_maze_line_pid(5.0,    6.0,    0,    -0.3,    0.3,    -4,     4);

// NEW PID CONTROLLER
PidController track_maze_line_pid(5.0,    6.0,    0,    -0.3,    0.3,    -4,     4);

// Global Variables
float degrees_ = 1.5708;  // 90 Degrees in Radians
float turn_distance = 0.089f*degrees_;
float ENCODER_DIST = 0.089f*2.0f*PI/29.86f/12.0f;
float last_turn = 0;
int dir[2] = {0 , 1};

// Main Control Task - Custom Mode for Maze Tracking
void MainControlTask::customMode(void)
{
    static float line_position;
    static float start_distance = 0.0f;
    static maze_mode_t maze_mode = TRACK_LINE;
    uint8_t qtr_state = 0;
    uint8_t num_qtr_on = 0;
    float delta_speed = 0.0f, left_speed_command = 0.0f, right_speed_command = 0.0f;
    float yaw_d;
    static float start_yaw = 0;
    float grid_distance = 0.2f;  // 20 cm distance
    static int grid_x = 0;
    static int grid_y = 0;

    for (uint8_t i = 0; i < 8; i++)                    debug_printf("Yaw Degrees = %i", yaw_d) ;
    {
        if (analog_.voltages[i] > QTR_THRESHOLD)
        {
            qtr_state |= 1<<i;
            num_qtr_on++;
        }
    }
    leds_task.requestNewLedGreenPattern(qtr_state);
    line_position = determine_line_pos(qtr_state, line_position, num_qtr_on);

    switch(maze_mode)
    {
        case TRACK_LINE:
                delta_speed = track_maze_line_pid.calculate(0.0f - line_position, delta_t_);
                left_speed_command = NOMINAL_SPEED + delta_speed;
                right_speed_command = NOMINAL_SPEED - delta_speed;
                start_yaw = odometry_.avg_distance;
                
                if ((num_qtr_on > 3) ||(num_qtr_on == 0))
                {
                    maze_mode = NODE;
                    start_distance = odometry_.avg_distance;
                }
                break;


        case NODE:
                left_speed_command = NOMINAL_SPEED;
                right_speed_command= NOMINAL_SPEED;
                // distance 0.003f
                if ((odometry_.avg_distance - start_distance) > 0.003f)
                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                    if (num_qtr_on == 8)
                    {
                        maze_mode = ADVANCE_LEFT;
                    }

                    else if ((analog_.voltages[7] > QTR_THRESHOLD) && (analog_.voltages[6] > QTR_THRESHOLD) &&
                            (analog_.voltages[0] < QTR_THRESHOLD) && (analog_.voltages[1] < QTR_THRESHOLD))
                    {
                        maze_mode = ADVANCE_LEFT;
                    }
                    else if ((analog_.voltages[7] > QTR_THRESHOLD) && (analog_.voltages[6] > QTR_THRESHOLD) &&
                             (analog_.voltages[0] > QTR_THRESHOLD) && (analog_.voltages[1] > QTR_THRESHOLD) &&
                             (analog_.voltages[2] < QTR_THRESHOLD) && (analog_.voltages[3] < QTR_THRESHOLD) &&
                             (analog_.voltages[4] < QTR_THRESHOLD) && (analog_.voltages[5] < QTR_THRESHOLD))
                    {
                        maze_mode = TERMINATION;
                    }

                    else
                    {
                        maze_mode = RIGHT_CHECK;
                    }
                break;
                }


        case RIGHT_CHECK:
                left_speed_command = NOMINAL_SPEED;
                right_speed_command= NOMINAL_SPEED;
                //distance 0.004f
                if ((odometry_.avg_distance - start_distance) > 0.004f)
                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                    if (num_qtr_on > 1)
                    {
                        maze_mode = TRACK_LINE;
                    }
                    else
                    {
                        maze_mode = ADVANCE_RIGHT;
                    }
                }
                break;

        case ADVANCE_LEFT:
                left_speed_command = NOMINAL_SPEED;
                right_speed_command= NOMINAL_SPEED;
                // distance 0.135f
                if ((odometry_.avg_distance - start_distance) > 0.125f)
                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                    maze_mode = TURN_LEFT;
                }
                break;

        case ADVANCE_RIGHT:
                left_speed_command = NOMINAL_SPEED;
                right_speed_command= NOMINAL_SPEED;
                // distance 0.131f
                if ((odometry_.avg_distance - start_distance) > 0.121f)
                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                    maze_mode = TURN_RIGHT;
                }
                break;

        case TURN_LEFT:
                left_speed_command = -NOMINAL_SPEED;
                right_speed_command = NOMINAL_SPEED;
                if (analog_.voltages[7] > QTR_THRESHOLD)

                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                    maze_mode = TRACK_LINE;

                }
                break;

        case TURN_RIGHT:
                left_speed_command = NOMINAL_SPEED;
                right_speed_command = -NOMINAL_SPEED;
                if (analog_.voltages[0] > QTR_THRESHOLD)
                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                    maze_mode = TRACK_LINE;
                }
                break;

        case TERMINATION:
                left_speed_command = NOMINAL_SPEED;
                right_speed_command= NOMINAL_SPEED;
                if ((odometry_.avg_distance - start_distance) > 0.135f)
                {
                    left_speed_command = 0.0f;
                    right_speed_command = 0.0f;
                }
                break;
    }

    float left_duty_command = left_speed_pid.calculate(left_speed_command - odometry_.left_speed, delta_t_);
    float right_duty_command = right_speed_pid.calculate(right_speed_command - odometry_.right_speed, delta_t_);
    motor_pwm_.left_duty = left_duty_command;
    motor_pwm_.right_duty = right_duty_command;
}

float determine_line_pos(uint8_t qtr_state, float last_line_pos, uint8_t num_qtr_on)
{
    static int8_t line_lost_counter = 0;
    float line_pos = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (qtr_state & (1<<i))
        {
            line_pos += (0.009525*i - 0.0333375);
        }
    }
    if (( num_qtr_on ) && (num_qtr_on < 3))
    {
        if (line_lost_counter) line_lost_counter -= 1;
        else line_pos = line_pos/num_qtr_on;
    }
    else
    {
        line_lost_counter = 9;
        line_pos = last_line_pos;
    }
    return (line_pos);
}
