#include "L1AdaptiveControl.hpp"

L1AdaptiveControl::L1AdaptiveControl() :
ModuleParams(nullptr),
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

L1AdaptiveControl::~L1AdaptiveControl()
{
perf_free(_loop_perf);
perf_free(_loop_interval_perf);
}

bool L1AdaptiveControl::init()
{
PX4_INFO("L1 adaptive control init");

// 10 Hz debug loop.
// At this stage we only subscribe and print uORB states.
ScheduleOnInterval(100_ms);

return true;
}

void L1AdaptiveControl::Run()
{
if (should_exit()) {
ScheduleClear();
exit_and_cleanup();
return;
}

perf_begin(_loop_perf);
perf_count(_loop_interval_perf);

if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
_has_local_position = true;
}

if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
_has_attitude = true;
}

if (_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)) {
_has_angular_velocity = true;
}

if (_vehicle_status_sub.update(&_vehicle_status)) {
_has_vehicle_status = true;
}

const hrt_abstime now_us = hrt_absolute_time();

if (now_us - _last_print_us > 1000000) {
print_debug_info();
_last_print_us = now_us;
}

perf_end(_loop_perf);
}

void L1AdaptiveControl::print_debug_info()
{
PX4_INFO("========== L1 adaptive uORB debug ==========");

if (_has_vehicle_status) {
const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

PX4_INFO("status: armed=%d arming_state=%u nav_state=%u failsafe=%d",
 (int)armed,
 (unsigned)_vehicle_status.arming_state,
 (unsigned)_vehicle_status.nav_state,
 (int)_vehicle_status.failsafe);

} else {
PX4_WARN("vehicle_status: no data yet");
}

if (_has_local_position) {
PX4_INFO("local_position valid: xy=%d z=%d v_xy=%d v_z=%d",
 (int)_vehicle_local_position.xy_valid,
 (int)_vehicle_local_position.z_valid,
 (int)_vehicle_local_position.v_xy_valid,
 (int)_vehicle_local_position.v_z_valid);

PX4_INFO("pos NED [m]: x=%.3f y=%.3f z=%.3f",
 (double)_vehicle_local_position.x,
 (double)_vehicle_local_position.y,
 (double)_vehicle_local_position.z);

PX4_INFO("vel NED [m/s]: vx=%.3f vy=%.3f vz=%.3f",
 (double)_vehicle_local_position.vx,
 (double)_vehicle_local_position.vy,
 (double)_vehicle_local_position.vz);

} else {
PX4_WARN("vehicle_local_position: no data yet");
}

if (_has_attitude) {
PX4_INFO("attitude q body_FRD_to_NED: w=%.4f x=%.4f y=%.4f z=%.4f",
 (double)_vehicle_attitude.q[0],
 (double)_vehicle_attitude.q[1],
 (double)_vehicle_attitude.q[2],
 (double)_vehicle_attitude.q[3]);

} else {
PX4_WARN("vehicle_attitude: no data yet");
}

if (_has_angular_velocity) {
PX4_INFO("angular_velocity body FRD [rad/s]: p=%.4f q=%.4f r=%.4f",
 (double)_vehicle_angular_velocity.xyz[0],
 (double)_vehicle_angular_velocity.xyz[1],
 (double)_vehicle_angular_velocity.xyz[2]);

} else {
PX4_WARN("vehicle_angular_velocity: no data yet");
}
}

int L1AdaptiveControl::task_spawn(int argc, char *argv[])
{
L1AdaptiveControl *instance = new L1AdaptiveControl();

if (instance) {
_object.store(instance);
_task_id = task_id_is_work_queue;

if (instance->init()) {
return PX4_OK;
}

} else {
PX4_ERR("alloc failed");
}

delete instance;
_object.store(nullptr);
_task_id = -1;

return PX4_ERROR;
}

int L1AdaptiveControl::print_status()
{
PX4_INFO("L1 adaptive control is running");

PX4_INFO("has local_position=%d attitude=%d angular_velocity=%d vehicle_status=%d",
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_vehicle_status);

perf_print_counter(_loop_perf);
perf_print_counter(_loop_interval_perf);

return 0;
}

int L1AdaptiveControl::custom_command(int argc, char *argv[])
{
return print_usage("unknown command");
}

int L1AdaptiveControl::print_usage(const char *reason)
{
if (reason) {
PX4_WARN("%s\n", reason);
}

PRINT_MODULE_DESCRIPTION(
R"DESCR_STR(
### Description
L1 adaptive control module skeleton for PX4 v1.17.0.

Current stage:
- Subscribe vehicle_local_position
- Subscribe vehicle_attitude
- Subscribe vehicle_angular_velocity
- Subscribe vehicle_status
- Print debug state only
- No control output is published
)DESCR_STR");

PRINT_MODULE_USAGE_NAME("l1_adaptive_control", "controller");
PRINT_MODULE_USAGE_COMMAND("start");
PRINT_MODULE_USAGE_COMMAND("status");
PRINT_MODULE_USAGE_COMMAND("stop");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

return 0;
}

extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[])
{
return L1AdaptiveControl::main(argc, argv);
}
