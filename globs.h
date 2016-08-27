// This is the only header file you should include to use globs.
// To define new objects you must modify this file and glob_types.h

#ifndef GLOBS_H_INCLUDED
#define GLOBS_H_INCLUDED

// Includes
#include <cstdint>
#include "glob_template.h"
#include "glob_types.h"

// Unique IDs given to globs are defined in this enumeration.
// Add an ID name to the enum for each new object.
// The bottom member of the enum should always be NUM_GLOBS which
// keeps track of the total number of objects (max number = 255)
typedef uint8_t glob_id_t;
enum
{
    GLO_ID_ASSERT_MESSAGE,
    GLO_ID_DEBUG_MESSAGE,
    GLO_ID_CAPTURE_DATA,
    GLO_ID_DRIVING_COMMAND,
    GLO_ID_CAPTURE_COMMAND,
    GLO_ID_STATUS_DATA,
    GLO_ID_MOTION_COMMANDS,
    GLO_ID_RAW_IMU,
    GLO_ID_ANALOG,
    GLO_ID_IMU,
    GLO_ID_ROLL_PITCH_YAW,
    GLO_ID_QUATERNION,
    GLO_ID_THETA_ZERO,
    GLO_ID_ODOMETRY,
    GLO_ID_MODES,
    GLO_ID_ROBOT_COMMAND,
    GLO_ID_MOTOR_PWM,
    GLO_ID_WAVE,
    GLO_ID_PID_PARAMS,
    GLO_ID_REQUEST,
    GLO_ID_TASK_TIMING,

    NUM_GLOBS,
};

// Macro used to allow globs.cpp to define the objects and avoid duplicate maintenance.
// In globs.cpp DEFINE_GLOBS forces the macro to define the objects.
// Elswhere it only declares the objects.
#ifndef DEFINE_GLOBS
#define GLOB(var_name, struct_type, id, num_instances, owner_task) \
    extern GlobTemplate<struct_type, num_instances, owner_task> var_name
#else
#define GLOB(var_name, struct_type, id, num_instances, owner_task) \
    GlobTemplate<struct_type, num_instances, owner_task> var_name(id)
#endif

// Forward declare task types for glob ownership.
class MainControlTask;
class StatusUpdateTask;
class TelemetryReceiveTask;
class ComplementaryFilterTask;
class ModesTask;
class TelemetrySendTask;

// Macro use:
// Argument 1: Glob variable name
// Argument 2: Name of struct (defined in glob_types.h)
// Argument 3: ID (must be unique, see enumeration above in CONSTANTS section above)
// Argument 4: How many instances of struct to hold (usually just one)
// Argument 5: The owner task allowed to publish the object
GLOB(glo_assert_message,       glo_assert_message_t,      GLO_ID_ASSERT_MESSAGE,       3,    TelemetrySendTask);
GLOB(glo_debug_message,        glo_debug_message_t,       GLO_ID_DEBUG_MESSAGE,        5,    TelemetrySendTask);
GLOB(glo_capture_data,         glo_capture_data_t,        GLO_ID_CAPTURE_DATA,         2001, MainControlTask); // 192K available RAM. This uses ~70K. Add 1 since instance 0 isn't used.
GLOB(glo_driving_command,      glo_driving_command_t,     GLO_ID_DRIVING_COMMAND,      1,    TelemetryReceiveTask);
GLOB(glo_capture_command,      glo_capture_command_t,     GLO_ID_CAPTURE_COMMAND,      1,    MainControlTask);
GLOB(glo_status_data,          glo_status_data_t,         GLO_ID_STATUS_DATA,          1,    StatusUpdateTask);
GLOB(glo_motion_commands,      glo_motion_commands_t,     GLO_ID_MOTION_COMMANDS,      1,    TelemetryReceiveTask);
GLOB(glo_raw_imu,              glo_raw_imu_t,             GLO_ID_RAW_IMU,              1,    ComplementaryFilterTask);
GLOB(glo_analog,               glo_analog_t,              GLO_ID_ANALOG,               1,    MainControlTask);
GLOB(glo_imu,                  glo_imu_t,                 GLO_ID_IMU,                  1,    ComplementaryFilterTask);
GLOB(glo_roll_pitch_yaw,       glo_roll_pitch_yaw_t,      GLO_ID_ROLL_PITCH_YAW,       1,    ComplementaryFilterTask);
GLOB(glo_quaternion,           glo_quaternion_t,          GLO_ID_QUATERNION,           1,    ComplementaryFilterTask);
GLOB(glo_theta_zero,           glo_theta_zero_t,          GLO_ID_THETA_ZERO,           1,    MainControlTask);
GLOB(glo_odometry,             glo_odometry_t,            GLO_ID_ODOMETRY,             1,    MainControlTask);
GLOB(glo_modes,                glo_modes_t,               GLO_ID_MODES,                1,    ModesTask);
GLOB(glo_robot_command,        glo_robot_command_t,       GLO_ID_ROBOT_COMMAND,        1,    ModesTask);
GLOB(glo_motor_pwm,            glo_motor_pwm_t,           GLO_ID_MOTOR_PWM,            1,    MainControlTask);
GLOB(glo_wave,                 glo_wave_t,                GLO_ID_WAVE,                 1,    MainControlTask);
GLOB(glo_pid_params,           glo_pid_params_t,          GLO_ID_PID_PARAMS,           NUM_PID_CONTROLLERS,  TelemetryReceiveTask);
GLOB(glo_request,              glo_request_t,             GLO_ID_REQUEST,              1,    TelemetryReceiveTask);
GLOB(glo_task_timing,          glo_task_timing_t,         GLO_ID_TASK_TIMING,          1,    TelemetrySendTask);

#endif // GLOBS_H_INCLUDED
