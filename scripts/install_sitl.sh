#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${1:-$repo_dir/PX4-Autopilot}"

if [[ ! -d "$px4_dir/src/modules" ]]; then
    echo "PX4 source not found: $px4_dir" >&2
    echo "Run: git submodule update --init --recursive" >&2
    exit 1
fi

copy_tree() {
    mkdir -p "$2"
    cp -a "$1/." "$2/"
}

apply_once() {
    if git -C "$1" apply --reverse --check "$2" >/dev/null 2>&1; then
        echo "Already applied: $(basename "$2")"
    else
        git -C "$1" apply --check "$2"
        git -C "$1" apply "$2"
    fi
}

copy_tree "$repo_dir/l1_adaptive_control" "$px4_dir/src/modules/l1_adaptive_control"
copy_tree "$repo_dir/l1_keyboard_throttle" "$px4_dir/src/modules/l1_keyboard_throttle"
copy_tree "$repo_dir/gz_plugins/motor_failure" "$px4_dir/src/modules/simulation/gz_plugins/motor_failure"
apply_once "$px4_dir" "$repo_dir/patches/px4-v1.17.0-gz-harmonic-integration.patch"
apply_once "$px4_dir" "$repo_dir/patches/px4-v1.17.0-motor-failure-allocation.patch"
apply_once "$px4_dir/Tools/simulation/gz" "$repo_dir/patches/px4-gazebo-models-x500-motor-failure.patch"

echo "Installed. Build with: cd $px4_dir && make px4_sitl gz_x500"
