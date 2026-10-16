#!/usr/bin/env bash
# =============================================================================
# Entrypoint for the Go Aero Virtual Jetson Orin NX SIL Container
# =============================================================================
set -e

# Source ROS 2 Humble base environment
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

# Source the built colcon workspace containing visual_obstacle_detection
if [ -f "/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/ros2_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Convenient one-shot self-test mode for CI / local verification
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    if [ "${1:-}" = "--self-test" ]; then
        echo "[jetson-sil] Running standalone synthetic ZED + obstacle detection self-test..."
        python3 /ros2_ws/sim_scripts/synthetic_zed_publisher.py --self-test
        python3 /ros2_ws/sim_scripts/latency_relay.py --self-test
        if [ -d "/ros2_ws/src/visual-obstacle-detection/visual_obstacle_detection/test" ]; then
            echo "[jetson-sil] Running visual_obstacle_detection upstream unit tests..."
            pytest -v /ros2_ws/src/visual-obstacle-detection/visual_obstacle_detection/test
        elif [ -d "/ros2_ws/src/visual-obstacle-detection/test" ]; then
            echo "[jetson-sil] Running visual_obstacle_detection upstream unit tests..."
            pytest -v /ros2_ws/src/visual-obstacle-detection/test
        fi
        exit 0
    fi

    if [ "$#" -eq 0 ]; then
        exec bash
    else
        exec "$@"
    fi
fi
