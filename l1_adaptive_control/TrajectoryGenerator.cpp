#include "TrajectoryGenerator.hpp"

bool TrajectoryGenerator::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

// Stage 4 placeholder:
// Generate the simplest possible trajectory:
// hold the first valid current position.
//
// In "make px4_sitl_default none", state_valid_for_control may stay false.
// In that case, we still copy current_position_ned into the output for debug printing,
// but output.valid remains false.
if (!_hold_initialized && input.state_valid_for_control && !input.failsafe) {
_hold_position_ned[0] = input.current_position_ned[0];
_hold_position_ned[1] = input.current_position_ned[1];
_hold_position_ned[2] = input.current_position_ned[2];

_hold_yaw = input.current_yaw;

_hold_initialized = true;
}

if (_hold_initialized) {
output.position_ned[0] = _hold_position_ned[0];
output.position_ned[1] = _hold_position_ned[1];
output.position_ned[2] = _hold_position_ned[2];

output.yaw = _hold_yaw;

} else {
// Debug fallback before a valid state exists.
output.position_ned[0] = input.current_position_ned[0];
output.position_ned[1] = input.current_position_ned[1];
output.position_ned[2] = input.current_position_ned[2];

output.yaw = input.current_yaw;
}

for (int i = 0; i < 3; i++) {
output.velocity_ned[i] = 0.f;
output.acceleration_ned[i] = 0.f;
output.jerk_ned[i] = 0.f;
output.snap_ned[i] = 0.f;
}

output.yaw_rate = 0.f;
output.yaw_accel = 0.f;

output.valid = input.state_valid_for_control && !input.failsafe;

_last_output = output;

// Return true only means the trajectory generator executed.
// It does NOT mean the trajectory is safe for control.
return true;
}

void TrajectoryGenerator::reset_hold_position()
{
_hold_initialized = false;
}
