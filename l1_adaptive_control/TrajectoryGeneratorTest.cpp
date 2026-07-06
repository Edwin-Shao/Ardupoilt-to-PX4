#include "TrajectoryGenerator.hpp"

#include <gtest/gtest.h>

namespace
{

TrajectoryGenerator::Input make_valid_input(hrt_abstime timestamp_us)
{
	TrajectoryGenerator::Input input{};
	input.timestamp_us = timestamp_us;
	input.current_position_ned[0] = 0.f;
	input.current_position_ned[1] = 0.f;
	input.current_position_ned[2] = 0.f;
	input.current_yaw = 0.f;
	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

TrajectoryGenerator::Output update_at(TrajectoryGenerator &generator, hrt_abstime timestamp_us,
				      bool manual_enabled = false, bool manual_valid = false, float stick = 0.f)
{
	TrajectoryGenerator::Input input = make_valid_input(timestamp_us);
	input.manual_height_control_enabled = manual_enabled;
	input.manual_height_control_valid = manual_valid;
	input.manual_height_stick = stick;

	TrajectoryGenerator::Output output{};
	EXPECT_TRUE(generator.update(input, output));
	EXPECT_TRUE(output.valid);
	return output;
}

} // namespace

TEST(TrajectoryGenerator, ManualHeightControlIsIgnoredBeforeHold)
{
	TrajectoryGenerator generator;

	const TrajectoryGenerator::Output output = update_at(generator, 1'000'000, true, true, 1.f);

	EXPECT_EQ(output.mode, TrajectoryGenerator::Mode::Takeoff);
	EXPECT_LT(output.position_ned[2], 0.f);
	EXPECT_GT(output.position_ned[2], -1.f);
}

TEST(TrajectoryGenerator, ManualHeightControlIntegratesNedVerticalTargetInHold)
{
	TrajectoryGenerator generator;

	update_at(generator, 0);
	const TrajectoryGenerator::Output hold = update_at(generator, 2'000'000);
	EXPECT_EQ(hold.mode, TrajectoryGenerator::Mode::Hold);
	EXPECT_FLOAT_EQ(hold.position_ned[2], -1.f);

	const TrajectoryGenerator::Output initialized = update_at(generator, 2'000'000, true, true, 0.f);
	EXPECT_FLOAT_EQ(initialized.position_ned[2], -1.f);
	EXPECT_FLOAT_EQ(initialized.velocity_ned[2], 0.f);

	const TrajectoryGenerator::Output climb = update_at(generator, 3'000'000, true, true, 1.f);
	EXPECT_FLOAT_EQ(climb.velocity_ned[2], -0.3f);
	EXPECT_FLOAT_EQ(climb.position_ned[2], -1.3f);

	const TrajectoryGenerator::Output descend = update_at(generator, 4'000'000, true, true, -1.f);
	EXPECT_FLOAT_EQ(descend.velocity_ned[2], 0.3f);
	EXPECT_FLOAT_EQ(descend.position_ned[2], -1.f);
}

TEST(TrajectoryGenerator, ManualHeightControlHoldsOnDeadzoneAndDisablesToDefaultHold)
{
	TrajectoryGenerator generator;

	update_at(generator, 0);
	update_at(generator, 2'000'000);
	update_at(generator, 2'000'000, true, true, 0.f);
	update_at(generator, 3'000'000, true, true, 1.f);

	const TrajectoryGenerator::Output deadzone = update_at(generator, 4'000'000, true, true, 0.05f);
	EXPECT_FLOAT_EQ(deadzone.velocity_ned[2], 0.f);
	EXPECT_FLOAT_EQ(deadzone.position_ned[2], -1.3f);

	const TrajectoryGenerator::Output disabled = update_at(generator, 5'000'000, false, false, 1.f);
	EXPECT_EQ(disabled.mode, TrajectoryGenerator::Mode::Hold);
	EXPECT_FLOAT_EQ(disabled.position_ned[2], -1.f);
	EXPECT_FLOAT_EQ(disabled.velocity_ned[2], 0.f);
}
