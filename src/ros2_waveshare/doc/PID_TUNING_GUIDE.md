# PID Tuning Guide for Single Motor System

## Overview

This guide provides step-by-step instructions for tuning the PID controller in the SingleMotorModel plugin for optimal velocity control performance.

## Table of Contents

1. [Understanding PID Control](#understanding-pid-control)
2. [Parameters](#parameters)
3. [Tuning Process](#tuning-process)
4. [Troubleshooting](#troubleshooting)
5. [Advanced Topics](#advanced-topics)

---

## Understanding PID Control

### What is PID?

PID (Proportional-Integral-Derivative) control is a feedback control algorithm that continuously calculates an error value as the difference between a desired setpoint and a measured process variable.

**Control Equation:**
```
output(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

Where:
- `e(t)` = error at time t = target - current
- `Kp` = Proportional gain
- `Ki` = Integral gain  
- `Kd` = Derivative gain

### How Each Term Works

#### Proportional (P)
- **Effect**: Responds to current error
- **Behavior**: Larger error → larger correction
- **Problem**: Cannot eliminate steady-state error (always some offset)
- **Tuning**: Start here first

#### Integral (I)
- **Effect**: Responds to accumulated error over time
- **Behavior**: Eliminates steady-state error
- **Problem**: Can cause overshoot and oscillation (integral windup)
- **Tuning**: Add after P is working

#### Derivative (D)
- **Effect**: Responds to rate of change of error
- **Behavior**: Dampens oscillations, predicts future error
- **Problem**: Sensitive to noise, can amplify measurement errors
- **Tuning**: Add last for fine-tuning

---

## Parameters

### PID Gains

Located in `config/velocity_converter_params.yaml` under `single_motor_model`:

```yaml
single_motor_model:
  # PID Gains
  kp: 1.0                    # Proportional gain
  ki: 0.1                    # Integral gain
  kd: 0.05                   # Derivative gain
```

### Limits

```yaml
  # Velocity Limits
  max_velocity: 1.0          # Maximum linear velocity (m/s)
  max_acceleration: 2.0      # Maximum acceleration (m/s²)
  
  # Anti-windup
  max_integral: 10.0         # Maximum integral term
```

### Physical Parameters

```yaml
  # Physical Parameters
  wheel_radius: 0.1          # Wheel radius in meters
  gear_ratio: 1.0            # Gear reduction ratio (motor:wheel)
```

**Important**: These must match your actual hardware!

---

## Tuning Process

### Step 0: Prerequisites

1. **Verify physical parameters are correct**
   ```bash
   # Measure wheel radius accurately
   # Check gear ratio from motor specs
   ```

2. **Ensure system is safe**
   ```bash
   # Start with low max_velocity (0.5 m/s)
   # Have emergency stop ready
   ```

3. **Launch system**
   ```bash
   ros2 launch ros2_waveshare single_motor_test.launch.py
   ```

### Step 1: Proportional Only (P-Control)

**Goal**: Get basic response without overshoot

1. **Set initial values**:
   ```yaml
   kp: 0.5
   ki: 0.0    # Disabled
   kd: 0.0    # Disabled
   ```

2. **Test and observe**:
   ```bash
   # Send velocity command
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.5, y: 0.0, z: 0.0}}"
   
   # Monitor feedback
   ros2 topic echo /motors/motor_1/feedback
   ```

3. **Tune Kp**:
   - **Too low**: Slow response, large steady-state error
   - **Too high**: Overshoot, oscillations
   - **Just right**: Quick response, settles near target (with small offset)

4. **Adjustment strategy**:
   ```
   Start: kp = 0.5
   If slow: kp = 1.0
   If oscillating: kp = 0.25
   Repeat until good response
   ```

**Expected behavior**: Motor reaches ~90% of target quickly but has steady-state error (e.g., target 0.5 m/s, actual 0.45 m/s).

### Step 2: Add Integral (PI-Control)

**Goal**: Eliminate steady-state error

1. **Set initial Ki**:
   ```yaml
   kp: 1.0      # From Step 1
   ki: 0.05     # Start small
   kd: 0.0      # Still disabled
   ```

2. **Test and observe**:
   - Watch for steady-state error to decrease
   - Monitor for overshoot and oscillations

3. **Tune Ki**:
   - **Too low**: Still has steady-state error
   - **Too high**: Overshoot, slow settling, oscillations
   - **Just right**: Reaches target exactly, minimal overshoot

4. **Watch for integral windup**:
   ```bash
   # If motor saturates or oscillates badly:
   # - Reduce ki
   # - Reduce max_integral
   ```

**Expected behavior**: Motor reaches exact target (0.5 m/s) but may have some overshoot.

### Step 3: Add Derivative (PID-Control)

**Goal**: Reduce overshoot and improve damping

1. **Set initial Kd**:
   ```yaml
   kp: 1.0      # From Step 1
   ki: 0.1      # From Step 2
   kd: 0.02     # Start very small
   ```

2. **Test and observe**:
   - Should see reduced overshoot
   - Faster settling time

3. **Tune Kd**:
   - **Too low**: Still overshoots
   - **Too high**: Jittery, slow response, sensitive to noise
   - **Just right**: Smooth approach, minimal overshoot

**Expected behavior**: Clean response with minimal overshoot, quick settling.

### Step 4: Fine-Tuning

1. **Test different velocities**:
   ```bash
   # Low speed
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
   
   # Medium speed
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
   
   # High speed
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8}}"
   ```

2. **Test velocity changes**:
   ```bash
   # Step response: 0 → 0.5 m/s
   # Observe rise time, overshoot, settling time
   
   # Reverse: 0.5 → 0 m/s
   # Should decelerate smoothly
   ```

3. **Adjust max_acceleration**:
   ```yaml
   max_acceleration: 2.0   # Smoother: lower value
                           # Faster: higher value
   ```

---

## Troubleshooting

### Problem: Motor doesn't move

**Possible causes**:
1. Physical parameters wrong (wheel_radius, gear_ratio)
2. Kp too low
3. No feedback from motor

**Solution**:
```bash
# Check feedback
ros2 topic echo /motors/motor_1/feedback

# Check commands
ros2 topic echo /motors/motor_1/command

# Increase Kp
kp: 2.0
```

### Problem: Oscillations

**Symptoms**: Motor velocity oscillates around target

**Causes**:
- Kp too high
- Ki too high
- System delay/lag

**Solution**:
```yaml
# Reduce gains by 50%
kp: 0.5   # was 1.0
ki: 0.05  # was 0.1
kd: 0.01  # was 0.02
```

### Problem: Overshoot

**Symptoms**: Motor exceeds target, then settles back

**Causes**:
- Ki too high
- Kd too low

**Solution**:
```yaml
# Reduce Ki, increase Kd
ki: 0.05   # was 0.1
kd: 0.05   # was 0.02
```

### Problem: Slow response

**Symptoms**: Takes too long to reach target

**Causes**:
- Kp too low
- max_acceleration too low

**Solution**:
```yaml
# Increase Kp and acceleration limit
kp: 2.0              # was 1.0
max_acceleration: 4.0 # was 2.0
```

### Problem: Steady-state error

**Symptoms**: Motor velocity stays slightly below/above target

**Causes**:
- Ki too low or zero

**Solution**:
```yaml
# Increase Ki
ki: 0.2   # was 0.1
```

### Problem: Integral windup

**Symptoms**: Large overshoot, slow to recover from disturbances

**Causes**:
- max_integral too high
- Ki too high during saturation

**Solution**:
```yaml
# Reduce integral limit
max_integral: 5.0   # was 10.0

# Or reduce Ki
ki: 0.05   # was 0.1
```

---

## Advanced Topics

### Ziegler-Nichols Method

A systematic approach to PID tuning:

1. **Set Ki = 0, Kd = 0**

2. **Increase Kp until sustained oscillation** (critical gain Ku)

3. **Measure oscillation period** Tu (seconds)

4. **Calculate PID gains**:
   ```
   Kp = 0.6 * Ku
   Ki = 2 * Kp / Tu
   Kd = Kp * Tu / 8
   ```

### Gain Scheduling

Adjust gains based on velocity:

```cpp
// Pseudocode - not implemented yet
if (target_velocity < 0.3) {
    // Low speed: need more aggressive control
    kp = 2.0;
} else {
    // High speed: smoother control
    kp = 1.0;
}
```

### Feed-Forward Control

Add velocity feed-forward to improve tracking:

```cpp
// Pseudocode - future enhancement
output = pid_output + (target_velocity / wheel_radius) * feedforward_gain;
```

### Velocity Profiling

Smooth acceleration/deceleration ramps:

```yaml
# Implemented via max_acceleration
max_acceleration: 2.0   # m/s²
```

---

## Example Tuning Session

### Initial State
```yaml
kp: 0.5
ki: 0.0
kd: 0.0
max_velocity: 1.0
max_acceleration: 2.0
wheel_radius: 0.1
```

### Observations
- Target: 0.5 m/s
- Actual: 0.42 m/s (steady-state error)
- Response time: 2 seconds (acceptable)

### Iteration 1: Increase Kp
```yaml
kp: 1.0   # Doubled
```
- Actual: 0.48 m/s (better, still has error)
- Slight overshoot to 0.52 m/s

### Iteration 2: Add Ki
```yaml
kp: 1.0
ki: 0.1
```
- Actual: 0.50 m/s (perfect!)
- Overshoot to 0.55 m/s, settles in 1 second

### Iteration 3: Add Kd
```yaml
kp: 1.0
ki: 0.1
kd: 0.05
```
- Actual: 0.50 m/s
- Overshoot reduced to 0.51 m/s
- Settles in 0.8 seconds
- **Final tuned values** ✓

---

## Monitoring Tools

### Real-time Plotting

```bash
# Install plotjuggler
sudo apt install ros-jazzy-plotjuggler-ros

# Launch
ros2 run plotjuggler plotjuggler

# Subscribe to:
# - /cmd_vel
# - /motors/motor_1/feedback
# - /motors/motor_1/command
```

### Topic Echo

```bash
# Watch target vs actual
ros2 topic echo /motors/motor_1/feedback --field velocity_rad_s
```

### RQT Graph

```bash
# Visualize node connections
rqt_graph
```

---

## References

1. [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)
2. [Ziegler-Nichols Tuning](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
3. [ROS2 Control Tutorials](https://control.ros.org/master/index.html)

---

## Revision History

- 2025-11-12: Initial version
- Author: Andrea Efficace
- Package: ros2_waveshare
