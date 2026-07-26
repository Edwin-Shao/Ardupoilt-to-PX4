# L1Quad Controller for PX4 v1.17.0

This repository contains the PX4 port of the L1 adaptive geometric controller
from [L1Quad](https://github.com/sigma-pi/L1Quad). It is intentionally kept
separate from the full PX4 source tree.

## Branches

- `main`: hardware firmware baseline and the verified L1 failure flight modes.
- `hitl`: Pixhawk 6C Mini SIH/jMAVSim hardware-in-the-loop setup.
- `sitl`: Gazebo SITL setup, renamed from `simulation`; all existing SITL,
  keyboard, QGC Joystick and motor-failure testing remains available.

## Included Features

- L1 adaptive geometric controller for PX4 v1.17.0.
- Takeoff, hover and trajectory control.
- RC, QGC Joystick and SITL keyboard input.
- Motor 1 failure injection with yaw control released.
- `L1 Failure`: Position-based failure mode that holds position and altitude.
- `L1 Altitude Failure`: Altitude-based failure mode with manual roll/pitch and
  throttle-controlled climb rate.
- PX4 integration patch and concise operating guide.

## Repository Layout

```text
l1_adaptive_control/       L1 controller module
l1_keyboard_throttle/      SITL keyboard input
patches/                   PX4 v1.17.0 integration patches
scripts/                   setup helpers
PX4-Autopilot/             PX4 source submodule when present on the branch
指导.txt                    SITL and HITL operating guide
```

## Use With PX4

Clone the branch required for the test:

```bash
git clone --recurse-submodules -b sitl https://github.com/Edwin-Shao/Ardupoilt-to-PX4.git
```

For HITL, replace `sitl` with `hitl`. Copy the two modules into
`PX4-Autopilot/src/modules/`, enable them in the target board configuration,
then apply the matching patch from `patches/`.

The current Ubuntu workspaces are:

```text
SITL: /home/edwin/PX4-Autopilot-v1.17.0
HITL: /home/edwin/PX4-Autopilot-v1.17.0-HITL
```

See [`指导.txt`](指导.txt) for the tested commands, jMAVSim startup order,
flight-mode configuration and troubleshooting.

## Safety

This is research flight-control software. Complete SITL and propeller-free
HITL tests before real flight, and always switch into a failure mode only after
a stable normal takeoff.
