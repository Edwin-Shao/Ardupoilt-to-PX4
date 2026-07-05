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

// 10 Hz for the first skeleton test.
// Later, after uORB subscriptions and controller logic are added,
// this can be changed to callback-driven or higher-rate scheduling.
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

const hrt_abstime now_us = hrt_absolute_time();

if (now_us - _last_print_us > 1000000) {
PX4_INFO("L1 adaptive control running");
_last_print_us = now_us;
}

perf_end(_loop_perf);
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

This first version only starts, schedules itself, and prints a heartbeat message.
No control output is published yet.
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
