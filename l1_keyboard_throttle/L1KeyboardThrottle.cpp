#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>

#include <math.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

extern "C" __EXPORT int l1_keyboard_throttle_main(int argc, char *argv[]);
extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[]);

namespace
{

static constexpr float THROTTLE_STEP = 0.2f;
static constexpr float THROTTLE_MIN = -1.0f;
static constexpr float THROTTLE_MAX = 1.0f;
static constexpr int PUBLISH_INTERVAL_MS = 50;

float constrain_throttle(float value)
{
	if (value > THROTTLE_MAX) {
		return THROTTLE_MAX;
	}

	if (value < THROTTLE_MIN) {
		return THROTTLE_MIN;
	}

	return value;
}

class RawTerminalGuard
{
public:
	RawTerminalGuard()
	{
		if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &_old_config) == 0) {
			_new_config = _old_config;
			_new_config.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
			_new_config.c_cc[VMIN] = 0;
			_new_config.c_cc[VTIME] = 0;
			_enabled = (tcsetattr(STDIN_FILENO, TCSANOW, &_new_config) == 0);
		}
	}

	~RawTerminalGuard()
	{
		if (_enabled) {
			tcsetattr(STDIN_FILENO, TCSANOW, &_old_config);
		}
	}

private:
	bool _enabled{false};
	struct termios _old_config {};
	struct termios _new_config {};
};

void publish_manual_control(uORB::Publication<manual_control_setpoint_s> &publisher, float throttle)
{
	manual_control_setpoint_s manual{};
	const hrt_abstime now = hrt_absolute_time();

	manual.timestamp = now;
	manual.timestamp_sample = now;
	manual.valid = true;
	manual.data_source = manual_control_setpoint_s::SOURCE_RC;
	manual.roll = 0.f;
	manual.pitch = 0.f;
	manual.yaw = 0.f;
	manual.throttle = throttle;
	manual.flaps = 0.f;
	manual.aux1 = 0.f;
	manual.aux2 = 0.f;
	manual.aux3 = 0.f;
	manual.aux4 = 0.f;
	manual.aux5 = 0.f;
	manual.aux6 = 0.f;
	manual.sticks_moving = (fabsf(throttle) > 0.001f);
	manual.buttons = 0;

	publisher.publish(manual);
}

void disable_l1_rc_control()
{
	char arg0[] = "l1_adaptive_control";
	char arg1[] = "rc_control";
	char arg2[] = "disable";
	char *argv[] = {arg0, arg1, arg2};

	const int ret = l1_adaptive_control_main(3, argv);

	if (ret != 0) {
		PX4_WARN("could not disable rc_control automatically");
		PX4_WARN("Please run: l1_adaptive_control rc_control disable");
	}
}

int run_keyboard_throttle()
{
	RawTerminalGuard terminal_guard;
	uORB::Publication<manual_control_setpoint_s> manual_control_pub{ORB_ID(manual_control_setpoint)};
	float throttle = 0.f;

	PX4_INFO("l1_keyboard_throttle started");
	PX4_INFO("keys: w +0.2, s -0.2, 0 reset+disable rc_control, q quit");
	PX4_INFO("throttle = %.1f", (double)throttle);

	while (true) {
		struct pollfd fds {};
		fds.fd = STDIN_FILENO;
		fds.events = POLLIN;

		const int poll_ret = poll(&fds, 1, PUBLISH_INTERVAL_MS);

		if (poll_ret > 0 && (fds.revents & POLLIN)) {
			char key = 0;

			while (read(STDIN_FILENO, &key, 1) == 1) {
				if (key == 'w' || key == 'W') {
					throttle = constrain_throttle(throttle + THROTTLE_STEP);
					PX4_INFO("throttle = %.1f", (double)throttle);

				} else if (key == 's' || key == 'S') {
					throttle = constrain_throttle(throttle - THROTTLE_STEP);
					PX4_INFO("throttle = %.1f", (double)throttle);

				} else if (key == '0') {
					throttle = 0.f;
					publish_manual_control(manual_control_pub, throttle);
					PX4_INFO("throttle = %.1f", (double)throttle);
					disable_l1_rc_control();

				} else if (key == 'q' || key == 'Q' || key == 0x03 || key == 0x1b) {
					throttle = 0.f;
					publish_manual_control(manual_control_pub, throttle);
					PX4_INFO("l1_keyboard_throttle stopped");
					return 0;
				}
			}
		}

		publish_manual_control(manual_control_pub, throttle);
	}
}

void print_usage()
{
	PX4_INFO("Usage: l1_keyboard_throttle start");
	PX4_INFO("       l1_keyboard_throttle help");
}

} // namespace

int l1_keyboard_throttle_main(int argc, char *argv[])
{
	if (argc < 2 || !strcmp(argv[1], "-h") || !strcmp(argv[1], "help")) {
		print_usage();
		return 0;
	}

	if (!strcmp(argv[1], "start")) {
		return run_keyboard_throttle();
	}

	print_usage();
	return 1;
}
