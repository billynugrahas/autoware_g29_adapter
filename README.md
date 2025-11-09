# Autoware G29 Adapter

A ROS 2 adapter node that bridges Autoware's steering control commands to the Logitech G29 racing wheel force feedback system.

## Overview

This package provides a seamless integration between Autoware's steering control interface and the Logitech G29 racing wheel, enabling real-time force feedback based on Autoware's autonomous driving commands. The adapter translates steering tire angles from Autoware into physical wheel positions for the G29, creating an immersive hardware-in-the-loop experience.

## Features

- **Real-time Steering Translation**: Converts Autoware steering commands to G29 wheel positions
- **Configurable Steering Ratio**: Adjustable ratio between tire angle and wheel angle
- **Direction Inversion**: Handles coordinate system differences between Autoware and G29
- **Angle Clamping**: Ensures wheel commands stay within safe physical limits
- **QoS Compatibility**: Uses appropriate QoS settings for reliable communication with Autoware

## Node Information

### Node Name
- `autoware_g29_adapter`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/api/control/command/steering` | [`autoware_adapi_v1_msgs/msg/SteeringCommand`](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/types/autoware_adapi_v1_msgs/msg/SteeringCommand/) | Steering commands from Autoware containing tire angle in radians |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ff_target` | `ros_g29_force_feedback/msg/ForceFeedback` | Force feedback commands for the G29 wheel |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steering_ratio` | double | `15.0` | Ratio between tire angle and wheel angle (typical automotive steering ratio) |
| `max_steering_angle_deg` | double | `300.0` | Maximum allowed wheel angle in degrees (prevents exceeding physical limits) |
| `default_torque` | double | `0.5` | Default torque value for force feedback (range: 0.0 to 1.0) |
| `invert_steering_direction` | bool | `true` | Inverts steering direction to match coordinate conventions |

### Parameter Details

#### `steering_ratio`
The steering ratio defines the relationship between the tire angle and the steering wheel angle. A ratio of 15.0 means the steering wheel must rotate 15 degrees to achieve 1 degree of tire angle. This value should match your vehicle's characteristics for realistic feedback.

#### `max_steering_angle_deg`
Physical limit for the steering wheel angle. The G29 wheel has a maximum rotation of ±450 degrees, but this parameter allows you to constrain it further (e.g., 300 degrees for more realistic automotive behavior).

#### `default_torque`
Controls the resistance/force applied to the wheel. Values range from 0.0 (no force) to 1.0 (maximum force). Adjust based on your preference and physical setup.

#### `invert_steering_direction`
Compensates for coordinate system differences:
- **Autoware convention**: Negative values = turn right (clockwise)
- **G29 convention**: Negative values = turn left (counter-clockwise)
- Setting this to `true` ensures the wheel turns in the correct direction

## How It Works

### Steering Conversion Pipeline

1. **Input**: Receives steering tire angle in radians from Autoware
2. **Conversion**: Converts radians to degrees
3. **Scaling**: Multiplies by steering ratio to get wheel angle
4. **Inversion**: Applies direction inversion if enabled
5. **Clamping**: Limits wheel angle to `max_steering_angle_deg`
6. **Normalization**: Converts to G29 position range (-1.0 to 1.0)
7. **Output**: Publishes force feedback command to G29

### Example Calculation

```
Input: steering_tire_angle = 0.1 radians (≈5.73°)
steering_ratio = 15.0

Step 1: Convert to degrees
  tire_angle_deg = 0.1 × (180/π) = 5.73°

Step 2: Apply steering ratio
  wheel_angle_deg = 5.73 × 15.0 = 85.95°

Step 3: Invert direction (if enabled)
  wheel_angle_deg = -85.95°

Step 4: Clamp to max angle (300°)
  wheel_angle_deg = -85.95° (no change, within limits)

Step 5: Normalize to G29 range (-1 to 1)
  normalized_position = -85.95 / 450 = -0.191

Output: position = -0.191, torque = 0.5
```

## Installation

### Prerequisites

- ROS 2 Humble or later
- [`ros-g29-force-feedback`](https://github.com/billynugrahas/autoware_g29_adapter) package
- Autoware message packages:
  ```bash
  sudo apt install ros-$ROS_DISTRO-autoware-adapi-v1-msgs ros-$ROS_DISTRO-autoware-msgs
  ```

### Build

```bash
cd ~/your_ros2_workspace
colcon build --packages-select autoware_g29_adapter
source install/setup.bash
```

## Usage

### Launch with Default Parameters

```bash
ros2 launch autoware_g29_adapter autoware_g29_adapter.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch autoware_g29_adapter autoware_g29_adapter.launch.py \
  steering_ratio:=17.0 \
  max_steering_angle_deg:=350.0 \
  default_torque:=0.7
```

### Run Node Directly

```bash
ros2 run autoware_g29_adapter autoware_g29_adapter_node \
  --ros-args \
  -p steering_ratio:=15.0 \
  -p max_steering_angle_deg:=300.0 \
  -p default_torque:=0.5 \
  -p invert_steering_direction:=true
```

## Configuration

The default configuration file is located at [`config/autoware_g29_adapter.yaml`](config/autoware_g29_adapter.yaml). Modify this file to adjust parameters for your specific setup.

## Integration with Autoware

This adapter is designed to work with Autoware's control API. Ensure that:

1. Autoware is publishing steering commands to `/api/control/command/steering`
2. The G29 force feedback node is subscribed to `/ff_target`
3. The G29 wheel is properly connected and configured

### System Architecture

```
┌──────────────┐         ┌─────────────────────┐         ┌─────────────────┐
│   Autoware   │ ──────> │ autoware_g29_adapter│ ──────> │ G29 FF Driver   │
│              │         │                     │         │                 │
│ Steering     │         │ Coordinate          │         │ Force           │
│ Controller   │         │ Transform           │         │ Feedback        │
└──────────────┘         └─────────────────────┘         └─────────────────┘
      |                            |                              |
      v                            v                              v
 SteeringCommand            Conversion Logic              Physical Wheel
  (tire angle)              (ratio, inversion)            (position/torque)
```

## Debugging

### Enable Debug Logging

```bash
ros2 run autoware_g29_adapter autoware_g29_adapter_node \
  --ros-args --log-level debug
```

Debug logs include detailed information about each steering command conversion.

### Monitor Topics

```bash
# Monitor incoming Autoware commands
ros2 topic echo /api/control/command/steering

# Monitor outgoing G29 commands
ros2 topic echo /ff_target

# Check message rates
ros2 topic hz /api/control/command/steering
ros2 topic hz /ff_target
```

## Troubleshooting

### Wheel Turns in Wrong Direction
- Check `invert_steering_direction` parameter
- Verify Autoware coordinate system configuration

### Wheel Movement Too Sensitive/Insensitive
- Adjust `steering_ratio` parameter
- Lower values = more sensitive
- Higher values = less sensitive

### Wheel Hits Physical Limits
- Reduce `max_steering_angle_deg` parameter
- Check if steering ratio is appropriate

### No Force Feedback
- Verify `default_torque` is greater than 0.0
- Check G29 force feedback node is running
- Ensure `/ff_target` topic is being published

## License

[Add your license information here]

## Author

[Add author information here]

## Contributing

[Add contribution guidelines here]

## See Also

- [ros-g29-force-feedback](../ros-g29-force-feedback/): G29 force feedback driver
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/): Official Autoware documentation