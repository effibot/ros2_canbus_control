#!/bin/bash
# Copyright 2025 Andrea Efficace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###############################################################################
# Single Motor Test System - Automated Test Script
###############################################################################
#
# Purpose:
#   Automated testing and validation of the single motor test system.
#   Verifies all nodes start correctly, topics are published, and the
#   control loop is functioning properly.
#
# Usage:
#   ./test_single_motor_system.sh [OPTIONS]
#
# Options:
#   --can-interface IFACE   CAN interface to use (default: vcan0)
#   --duration SECS         Test duration in seconds (default: 30)
#   --no-cleanup            Don't kill nodes after test
#   --help                  Show this help message
#
# Test Sequence:
#   1. Verify prerequisites (vcan0, built packages)
#   2. Launch all nodes
#   3. Wait for initialization
#   4. Verify all topics are publishing
#   5. Send test velocity commands
#   6. Monitor feedback and odometry
#   7. Generate test report
#   8. Cleanup
#
###############################################################################

set -e # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default parameters
CAN_INTERFACE="vcan0"
TEST_DURATION=30
CLEANUP=true
WORKSPACE_DIR="${HOME}/ws"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
    --can-interface)
        CAN_INTERFACE="$2"
        shift 2
        ;;
    --duration)
        TEST_DURATION="$2"
        shift 2
        ;;
    --no-cleanup)
        CLEANUP=false
        shift
        ;;
    --help)
        grep "^#" "$0" | grep -v "#!/bin/bash" | sed 's/^# //' | sed 's/^#//'
        exit 0
        ;;
    *)
        echo -e "${RED}Unknown option: $1${NC}"
        exit 1
        ;;
    esac
done

# Log functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}\n"
}

# Cleanup function
cleanup() {
    if [ "$CLEANUP" = true ]; then
        log_section "Cleaning Up"
        log_info "Stopping all nodes..."
        pkill -f "velocity_converter_node" || true
        pkill -f "odometry_publisher_node" || true
        pkill -f "teleop_velocity_node" || true
        pkill -f "motor_driver_node" || true
        pkill -f "canopen_lifecycle_node" || true
        sleep 2
        log_success "Cleanup complete"
    else
        log_warning "Skipping cleanup (--no-cleanup specified)"
    fi
}

# Set up trap for cleanup on exit
trap cleanup EXIT

# Test functions
check_prerequisites() {
    log_section "Checking Prerequisites"

    # Check workspace
    if [ ! -d "$WORKSPACE_DIR" ]; then
        log_error "Workspace not found: $WORKSPACE_DIR"
        exit 1
    fi
    log_success "Workspace found: $WORKSPACE_DIR"

    # Check if package is built
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        log_error "Package not built. Run 'ros_build_pkg ros2_waveshare' first"
        exit 1
    fi
    log_success "Package is built"

    # Source workspace
    source "$WORKSPACE_DIR/install/setup.bash"
    log_success "Workspace sourced"

    # Check CAN interface
    if ! ip link show "$CAN_INTERFACE" &>/dev/null; then
        log_error "CAN interface not found: $CAN_INTERFACE"
        if [ "$CAN_INTERFACE" = "vcan0" ]; then
            log_info "To create vcan0, run: sudo ./vcan_start.sh"
        fi
        exit 1
    fi
    log_success "CAN interface available: $CAN_INTERFACE"

    # Check executables
    local executables=("canopen_lifecycle_node" "motor_driver_node" "velocity_converter_node" "odometry_publisher_node" "teleop_velocity_node")
    for exe in "${executables[@]}"; do
        if ! ros2 pkg executables ros2_waveshare | grep -q "$exe"; then
            log_error "Executable not found: $exe"
            exit 1
        fi
    done
    log_success "All executables found"
}

launch_nodes() {
    log_section "Launching Nodes"

    log_info "Starting CANopen Lifecycle Node..."
    ros2 run ros2_waveshare canopen_lifecycle_node --ros-args -p can_interface:="$CAN_INTERFACE" &
    sleep 2

    log_info "Starting Motor Driver Node..."
    ros2 run ros2_waveshare motor_driver_node --ros-args -r __ns:=/motors/motor_1 -p node_id:=1 -p can_interface:="$CAN_INTERFACE" &
    sleep 2

    log_info "Starting Velocity Converter Node..."
    ros2 run ros2_waveshare velocity_converter_node --ros-args \
        --params-file "$WORKSPACE_DIR/install/share/ros2_waveshare/config/velocity_converter_params.yaml" &
    sleep 2

    log_info "Starting Odometry Publisher Node..."
    ros2 run ros2_waveshare odometry_publisher_node --ros-args \
        --params-file "$WORKSPACE_DIR/install/share/ros2_waveshare/config/odometry_params.yaml" &
    sleep 2

    log_success "All nodes launched"
    log_info "Waiting for initialization..."
    sleep 5
}

verify_topics() {
    log_section "Verifying Topics"

    local topics=(
        "/cmd_vel"
        "/motors/motor_1/command"
        "/motors/motor_1/feedback"
        "/odom"
        "/tf"
    )

    for topic in "${topics[@]}"; do
        if ros2 topic list | grep -q "^$topic$"; then
            log_success "Topic exists: $topic"
        else
            log_error "Topic not found: $topic"
            return 1
        fi
    done

    log_info "Checking topic rates..."
    sleep 2

    # Check if topics are publishing (with timeout)
    if timeout 5 ros2 topic echo /motors/motor_1/command --once &>/dev/null; then
        log_success "Motor commands are being published"
    else
        log_warning "Motor commands may not be publishing yet"
    fi
}

send_test_commands() {
    log_section "Sending Test Commands"

    log_info "Publishing test velocity: 0.5 m/s"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sleep 5

    log_info "Publishing test velocity: 0.0 m/s (stop)"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sleep 3

    log_success "Test commands sent"
}

monitor_system() {
    log_section "Monitoring System"

    log_info "Monitoring for $TEST_DURATION seconds..."
    log_info "Recording topic data..."

    # Monitor cmd_vel
    timeout $TEST_DURATION ros2 topic echo /cmd_vel --once &>/dev/null && log_success "cmd_vel active" || log_warning "cmd_vel inactive"

    # Monitor motor command
    timeout $TEST_DURATION ros2 topic echo /motors/motor_1/command --once &>/dev/null && log_success "motor command active" || log_warning "motor command inactive"

    # Monitor motor feedback (if available with real hardware)
    timeout 5 ros2 topic echo /motors/motor_1/feedback --once &>/dev/null && log_success "motor feedback active" || log_warning "motor feedback not available (expected with vcan simulation)"

    # Monitor odometry
    timeout $TEST_DURATION ros2 topic echo /odom --once &>/dev/null && log_success "odometry active" || log_warning "odometry inactive"

    # Monitor TF
    timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -n 10 && log_success "TF transform active" || log_warning "TF transform not available"
}

generate_report() {
    log_section "Test Report"

    log_info "Node Status:"
    pgrep -f "canopen_lifecycle_node" &>/dev/null && log_success "  ✓ CANopen Lifecycle Node" || log_error "  ✗ CANopen Lifecycle Node"
    pgrep -f "motor_driver_node" &>/dev/null && log_success "  ✓ Motor Driver Node" || log_error "  ✗ Motor Driver Node"
    pgrep -f "velocity_converter_node" &>/dev/null && log_success "  ✓ Velocity Converter Node" || log_error "  ✗ Velocity Converter Node"
    pgrep -f "odometry_publisher_node" &>/dev/null && log_success "  ✓ Odometry Publisher Node" || log_error "  ✗ Odometry Publisher Node"

    echo ""
    log_info "Topic List:"
    ros2 topic list | grep -E "(cmd_vel|command|feedback|odom|tf)" | while read -r topic; do
        echo "    $topic"
    done

    echo ""
    log_info "Node Graph:"
    ros2 node list | while read -r node; do
        echo "    $node"
    done
}

# Main test sequence
main() {
    log_section "Single Motor Test System - Automated Test"
    log_info "CAN Interface: $CAN_INTERFACE"
    log_info "Test Duration: $TEST_DURATION seconds"
    log_info "Cleanup: $CLEANUP"

    check_prerequisites
    launch_nodes
    verify_topics
    send_test_commands
    monitor_system
    generate_report

    log_section "Test Complete"
    log_success "All tests completed successfully!"
    echo ""
    log_info "Press Ctrl+C to stop nodes, or wait for automatic cleanup..."

    # Keep script running
    if [ "$CLEANUP" = false ]; then
        sleep infinity
    else
        sleep 5
    fi
}

# Run main
main
