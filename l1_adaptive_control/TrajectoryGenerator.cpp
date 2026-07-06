#include "TrajectoryGenerator.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

bool TrajectoryGenerator::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

set_zero_derivatives(output);

// Safety gate:
// If the PX4 state is not valid for control or the vehicle is disarmed, only
// mirror current position for debug and reset the trajectory start point.
// Do not generate a valid trajectory.
if (!input.state_valid_for_control || !input.armed || input.failsafe) {
reset();

output.position_ned[0] = input.current_position_ned[0];
output.position_ned[1] = input.current_position_ned[1];
output.position_ned[2] = input.current_position_ned[2];

output.yaw = input.current_yaw;
output.elapsed_time_s = 0.f;
output.mode = Mode::WaitForValidState;
output.valid = false;

_last_output = output;
return true;
}

// Initialize trajectory start point on the first valid state.
if (!_initialized) {
_start_time_us = input.timestamp_us;
_last_update_us = input.timestamp_us;

_start_position_ned[0] = input.current_position_ned[0];
_start_position_ned[1] = input.current_position_ned[1];
_start_position_ned[2] = input.current_position_ned[2];

_takeoff_target_position_ned[0] = _start_position_ned[0];
_takeoff_target_position_ned[1] = _start_position_ned[1];

// NED convention:
// z becomes more negative when the vehicle moves upward.
_takeoff_target_position_ned[2] = _start_position_ned[2] - TAKEOFF_HEIGHT_M;

_start_yaw = input.current_yaw;

_initialized = true;
}

const float elapsed_s = math::max((input.timestamp_us - _start_time_us) * 1e-6f, 0.f);
output.elapsed_time_s = elapsed_s;

output.yaw = _start_yaw;
output.yaw_rate = 0.f;
output.yaw_accel = 0.f;

if (elapsed_s < TAKEOFF_DURATION_S) {
output.mode = Mode::Takeoff;
_manual_hold_initialized = false;

const float s = math::constrain(elapsed_s / TAKEOFF_DURATION_S, 0.f, 1.f);

// Smooth cubic trajectory:
// h(s) = 3s^2 - 2s^3
// h_dot = (6s - 6s^2) / T
// h_ddot = (6 - 12s) / T^2
// h_jerk = -12 / T^3
const float h = 3.f * s * s - 2.f * s * s * s;
const float h_dot = (6.f * s - 6.f * s * s) / TAKEOFF_DURATION_S;
const float h_ddot = (6.f - 12.f * s) / (TAKEOFF_DURATION_S * TAKEOFF_DURATION_S);
const float h_jerk = -12.f / (TAKEOFF_DURATION_S * TAKEOFF_DURATION_S * TAKEOFF_DURATION_S);

for (int i = 0; i < 3; i++) {
const float delta = _takeoff_target_position_ned[i] - _start_position_ned[i];

output.position_ned[i] = _start_position_ned[i] + delta * h;
output.velocity_ned[i] = delta * h_dot;
output.acceleration_ned[i] = delta * h_ddot;
output.jerk_ned[i] = delta * h_jerk;
output.snap_ned[i] = 0.f;
}

} else {
output.mode = Mode::Hold;

if (input.manual_height_control_enabled) {
update_manual_hold_target(input, output);

} else {
_manual_hold_initialized = false;
set_hold_position(output, _takeoff_target_position_ned);
}
}

_last_update_us = input.timestamp_us;
output.valid = true;

_last_output = output;

return true;
}

void TrajectoryGenerator::reset()
{
_initialized = false;
_start_time_us = 0;
_last_update_us = 0;
_manual_hold_initialized = false;
}

void TrajectoryGenerator::set_zero_derivatives(Output &output)
{
for (int i = 0; i < 3; i++) {
output.velocity_ned[i] = 0.f;
output.acceleration_ned[i] = 0.f;
output.jerk_ned[i] = 0.f;
output.snap_ned[i] = 0.f;
}

output.yaw_rate = 0.f;
output.yaw_accel = 0.f;
}

void TrajectoryGenerator::set_hold_position(Output &output, const float position_ned[3])
{
output.position_ned[0] = position_ned[0];
output.position_ned[1] = position_ned[1];
output.position_ned[2] = position_ned[2];

set_zero_derivatives(output);
}

void TrajectoryGenerator::update_manual_hold_target(const Input &input, Output &output)
{
if (!_manual_hold_initialized) {
_manual_hold_position_ned[0] = _takeoff_target_position_ned[0];
_manual_hold_position_ned[1] = _takeoff_target_position_ned[1];
_manual_hold_position_ned[2] = _takeoff_target_position_ned[2];
_manual_hold_initialized = true;
_last_update_us = input.timestamp_us;
}

const float dt = math::constrain((input.timestamp_us - _last_update_us) * 1e-6f, 0.f, 0.1f);
float stick = input.manual_height_control_valid ? math::constrain(input.manual_height_stick, -1.f, 1.f) : 0.f;

if (fabsf(stick) < MANUAL_HEIGHT_DEADZONE) {
stick = 0.f;
}

const float target_vz_ned = -stick * MANUAL_MAX_CLIMB_RATE_M_S;
_manual_hold_position_ned[2] += target_vz_ned * dt;

const float min_z_ned = _start_position_ned[2] - MANUAL_MAX_HEIGHT_M;
const float max_z_ned = _start_position_ned[2] - MANUAL_MIN_HEIGHT_M;
_manual_hold_position_ned[2] = math::constrain(_manual_hold_position_ned[2], min_z_ned, max_z_ned);

output.position_ned[0] = _manual_hold_position_ned[0];
output.position_ned[1] = _manual_hold_position_ned[1];
output.position_ned[2] = _manual_hold_position_ned[2];

set_zero_derivatives(output);
output.velocity_ned[2] = target_vz_ned;
}
