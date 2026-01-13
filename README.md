# BOT4 - VEX V5 Competition Robot

A VEX V5 competition robot control system featuring tank drive, PID-based autonomous navigation, Pure Pursuit path following, and Monte Carlo Localization using distance sensors.

## Features

- **Tank Drive Control** - Dual joystick control with exponential curve for fine control
- **PID Controllers** - Smooth, accurate movements for autonomous routines
- **Pure Pursuit Path Following** - Curved path navigation with dynamic speed control
- **Monte Carlo Localization (MCL)** - Real-time position estimation using 8 distance sensors
- **Dual IMU Fusion** - Averaged heading from two inertial sensors for improved accuracy
- **Motor Temperature Monitoring** - On-demand temperature display on controller screen
- **Modular Architecture** - Clean separation between hardware config, control logic, and autonomous routines

## Hardware Configuration

### Motors (11 total)
| Port | Motor | Gear Ratio | Purpose |
|------|-------|------------|---------|
|(ports are placeholder, replace with your actual ports)|
| 1-3| Left A/B/C | 6:1 (600 RPM) | Left drivetrain |
| 4-6 | Right A/B/C | 6:1 (600 RPM) | Right drivetrain |
| 7 | Upper | 18:1 (200 RPM) | Upper intake/scoring |
| 8 | Mid | 18:1 (200 RPM) | Middle intake/scoring |
| 9 | Lower | 6:1 (600 RPM) | Lower intake |

### Sensors
| Port | Sensor | Purpose |
|------|--------|---------|
|(ports are placeholder)|
| 10-11 | Inertial (x2) | Heading (dual-fused) |
| 12-19 | Distance (x8) | MCL localization |

### Pneumatics
| 3-Wire Port | Device |
|-------------|--------|
|(placeholder ports)|
| A | Arm piston |
| B | Match load piston |

## Project Structure

```
PROJECT2/
├── include/
│   ├── bot.hpp           # Master include (aggregates all headers)
│   ├── robot_config.hpp  # Hardware declarations & IMU fusion class
│   ├── constants.hpp     # Tuning constants, field map, PID gains
│   ├── types.hpp         # Data structures (Particle, Waypoint, etc.)
│   ├── utils.hpp         # Math helpers (clamp, wrap, curve, etc.)
│   ├── pid.hpp           # PID controller class
│   ├── drivetrain.hpp    # Drivetrain class with drive/turn/path methods
│   ├── location.hpp      # MCL localization class
│   ├── buttons.hpp       # Button handler declarations
│   └── auton.hpp         # Autonomous routine declarations
├── src/
│   ├── main.cpp          # Entry point, competition callbacks
│   ├── robot_config.cpp  # Hardware definitions
│   ├── pid.cpp           # PID implementation
│   ├── drivetrain.cpp    # Drive control & Pure Pursuit
│   ├── location.cpp      # MCL particle filter
│   ├── buttons.cpp       # Button handler implementations
│   └── auton.cpp         # Autonomous routines
├── vex/                  # VEX build system files
└── makefile
```

## Controls

### Driver Controls (Tank Drive)
| Input | Action |
|-------|--------|
| Left Stick | Left side speed |
| Right Stick | Right side speed |
|(controls are based on joystick distance from center, not joystick y values)|

## Documentation

### namespace bot


## License

This project is for VEX Robotics Competition use.

