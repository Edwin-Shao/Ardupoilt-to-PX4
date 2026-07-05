#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class TrajectoryGenerator
{
public:
struct Input {
hrt_abstime timestamp_us{0};

float current_position_ned[3]{0.f, 0.f, 0.f}; // x, y, z in NED [m]
float current_yaw{0.f};

bool state_valid_for_control{false};
bool armed{false};
bool failsafe{false};
uint8_t nav_state{0};
};

struct Output {
hrt_abstime timestamp_us{0};

float position_ned[3]{0.f, 0.f, 0.f};
float velocity_ned[3]{0.f, 0.f, 0.f};
float acceleration_ned[3]{0.f, 0.f, 0.f};
float jerk_ned[3]{0.f, 0.f, 0.f};
float snap_ned[3]{0.f, 0.f, 0.f};

float yaw{0.f};
float yaw_rate{0.f};
float yaw_accel{0.f};

bool valid{false};
};

TrajectoryGenerator() = default;
~TrajectoryGenerator() = default;

bool update(const Input &input, Output &output);

void reset_hold_position();

const Input &last_input() const { return _last_input; }
const Output &last_output() const { return _last_output; }

private:
bool _hold_initialized{false};

float _hold_position_ned[3]{0.f, 0.f, 0.f};
float _hold_yaw{0.f};

Input _last_input{};
Output _last_output{};
};
