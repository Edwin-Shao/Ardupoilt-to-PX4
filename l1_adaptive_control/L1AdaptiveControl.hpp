#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

class L1AdaptiveControl :
public ModuleBase<L1AdaptiveControl>,
public ModuleParams,
public px4::ScheduledWorkItem
{
public:
L1AdaptiveControl();
~L1AdaptiveControl() override;

static int task_spawn(int argc, char *argv[]);
static int custom_command(int argc, char *argv[]);
static int print_usage(const char *reason = nullptr);

bool init();

int print_status() override;

private:
void Run() override;

perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

hrt_abstime _last_print_us{0};
};
