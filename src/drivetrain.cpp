#include "drivetrain.hpp"
#include "auton.hpp"

Drivetrain::Drivetrain(vex::motor_group& left_motor, vex::motor_group& right_motor,
    bot::sensors::inertial_group& imu) : 
    _left_motor(left_motor), _right_motor(right_motor),
    _wheel_diameter(WHEEL_DIAMETER), _track_width(TRACK_WIDTH), _gear_ratio(GEAR_RATIO),
    _drive_kp(DRIVE_KP), _drive_ki(DRIVE_KI), _drive_kd(DRIVE_KD),
    _turn_kp(TURN_KP), _turn_ki(TURN_KI), _turn_kd(TURN_KD),
    _max_voltage(MAX_VOLTAGE),
    _max_accel(MAX_ACCEL),
    _imu(imu),
    _drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _left_drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _right_drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _turn_pid(0.9, 0.0, 0.07),
    _left_turn_pid(TURN_KP, TURN_KI, TURN_KD),
    _right_turn_pid(TURN_KP, TURN_KI, TURN_KD),
    _left_arc_pid(ARC_KP, ARC_KI, ARC_KD),
    _right_arc_pid(ARC_KP, ARC_KI, ARC_KD),
    _heading_pid(K_HEADING, 0.0, 0.5),
    _pp_k_heading(K_HEADING),
    _pp_k_slow(K_SLOW),
    _pp_max_curv_speed_factor(MAX_CURV_SPEED_FACTOR),
    _pp_min_speed_pct(MIN_SPEED_PCT)
{}

void Drivetrain::drive_for(double distance, double timeout, double speed_limit) {
    _left_motor.setStopping(vex::brakeType::brake);
    _right_motor.setStopping(vex::brakeType::brake);
    
    double targetDegrees = helpers::mmToDegrees(distance);
    
    double leftStartPos = _left_motor.position(degrees);
    double rightStartPos = _right_motor.position(degrees);
    double leftTargetPos = leftStartPos + targetDegrees;
    double rightTargetPos = rightStartPos + targetDegrees;
    
    _left_drive_pid.reset();
    _right_drive_pid.reset();
    
    double lastLeftOutput = 0.0;
    double lastRightOutput = 0.0;

    double leftCurrentPos, rightCurrentPos, leftError, rightError, leftOutput, rightOutput;
    
    double start_time = bot::Brain.Timer.time(vex::msec);
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        leftCurrentPos = _left_motor.position(degrees);
        rightCurrentPos = _right_motor.position(degrees);
        
        leftError = leftTargetPos - leftCurrentPos;
        rightError = rightTargetPos - rightCurrentPos;
        
        leftOutput = _left_drive_pid.compute(leftTargetPos, leftCurrentPos, 0.02);
        rightOutput = _right_drive_pid.compute(rightTargetPos, rightCurrentPos, 0.02);
        
        // Clamp to speed limit
        if (leftOutput > speed_limit) leftOutput = speed_limit;
        if (leftOutput < -speed_limit) leftOutput = -speed_limit;
        if (rightOutput > speed_limit) rightOutput = speed_limit;
        if (rightOutput < -speed_limit) rightOutput = -speed_limit;
        
        // Slew rate limiting: cap how fast output can change (prevents tipping)
        if (leftOutput - lastLeftOutput > _max_accel) leftOutput = lastLeftOutput + _max_accel;
        if (leftOutput - lastLeftOutput < -_max_accel) leftOutput = lastLeftOutput - _max_accel;
        if (rightOutput - lastRightOutput > _max_accel) rightOutput = lastRightOutput + _max_accel;
        if (rightOutput - lastRightOutput < -_max_accel) rightOutput = lastRightOutput - _max_accel;
        
        lastLeftOutput = leftOutput;
        lastRightOutput = rightOutput;
        
        _left_motor.spin(forward, leftOutput, percent);
        _right_motor.spin(forward, rightOutput, percent);
        
        if (std::abs(leftError) < 5 && std::abs(rightError) < 5) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }
        vex::task::sleep(20);
    }
    _left_motor.stop();
    _right_motor.stop();
    return;
}

void Drivetrain::turn_to_heading(double heading, double timeout, double speed_limit) {
    _left_motor.setStopping(vex::brakeType::brake);
    _right_motor.setStopping(vex::brakeType::brake);

    // convert heading from user convention (0° = +Y) to code convention (0° = +X)
    double targetHeading = heading + 90.0;
    if (targetHeading > 180.0) targetHeading -= 360.0;
    if (targetHeading <= -180.0) targetHeading += 360.0;

    _left_turn_pid.reset();
    //double lastOutput = 0.0;
    double start_time = bot::Brain.Timer.time(vex::msec);

    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        double currentHeading = _imu.get_heading();

        double error = helpers::angular_difference(currentHeading, targetHeading);
        
        // trick: (double setpoint = error, double input = 0.0) sets error in PID automatically to error.
        double output = _left_turn_pid.compute(error, 0.0, 0.02);

        output = math::clamp(output, -speed_limit, speed_limit);
        
        //cap how fast output can change (prevents tipping)
        //if (output - lastOutput > _max_accel) output = lastOutput + _max_accel;
        //if (output - lastOutput < -_max_accel) output = lastOutput - _max_accel;
        //lastOutput = output;

        _left_motor.spin(forward, output, percent);
        _right_motor.spin(reverse, output, percent);
        
        if (std::abs(error) < 1.0) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }
        vex::task::sleep(20);
    }
    _left_motor.stop();
    _right_motor.stop();
}

void Drivetrain::turn_to_heading(double heading, double timeout, double speed_limit, bool settle) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    int settle_count = 0;
    _left_motor.setPosition(0, vex::degrees);
    _right_motor.setPosition(0, vex::degrees);
    _turn_pid.reset();
    double current_heading, heading_error, output, left_speed, right_speed;
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        current_heading = bot::sensors::left_imu.heading(vex::degrees);
        heading_error = helpers::angular_difference(current_heading, heading);
        output = _turn_pid.compute(heading_error, 0.0, 0.02);
        output = math::clamp(output, -speed_limit, speed_limit);
        left_speed = output * 0.4;
        right_speed = -output * 0.4;
        _left_motor.spin(forward, left_speed, vex::voltageUnits::volt);
        _right_motor.spin(forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(heading_error) < 1.0) {
            if (settle) {
                settle_count++;
                if (settle_count >= 3) {
                    break;
                }
            } else {
                break;
            }
        } else {
            settle_count = 0;
        }
        vex::task::sleep(20);
    }
    brake();
    _left_motor.stop();
    _right_motor.stop();
    coast();
}

void Drivetrain::drive_arc(double radius, double angle, double timeout, double speed_limit) {
    // Validate inputs
    if (std::abs(radius) < 1.0) return; // Invalid radius
    if (std::abs(angle) < 0.1) return; // Invalid angle

    _left_motor.setStopping(vex::brakeType::brake);
    _right_motor.setStopping(vex::brakeType::brake);
    
    double leftRadius = radius + (_track_width / 2.0);
    double rightRadius = radius - (_track_width / 2.0);
    double leftDistance = (leftRadius * angle * M_PI) / 180.0;
    double rightDistance = (rightRadius * angle * M_PI) / 180.0;
    
    double leftTargetDegrees = helpers::mmToDegrees(leftDistance);
    double rightTargetDegrees = helpers::mmToDegrees(rightDistance);
    double leftStartPos = _left_motor.position(degrees);
    double rightStartPos = _right_motor.position(degrees);
    double leftTargetPos = leftStartPos + leftTargetDegrees;
    double rightTargetPos = rightStartPos + rightTargetDegrees;
    
    _left_arc_pid.reset();
    _right_arc_pid.reset();
    
    double lastLeftOutput = 0.0;
    double lastRightOutput = 0.0;
    
    double start_time = bot::Brain.Timer.time(vex::msec);
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        double leftCurrentPos = _left_motor.position(vex::degrees);
        double rightCurrentPos = _right_motor.position(vex::degrees);
        
        double leftError = leftTargetPos - leftCurrentPos;
        double rightError = rightTargetPos - rightCurrentPos;
        
        double leftOutput = _left_arc_pid.compute(leftTargetPos, leftCurrentPos, 0.02);
        double rightOutput = _right_arc_pid.compute(rightTargetPos, rightCurrentPos, 0.02);
        
        // Clamp to speed limit
        if (leftOutput > speed_limit) leftOutput = speed_limit;
        if (leftOutput < -speed_limit) leftOutput = -speed_limit;
        if (rightOutput > speed_limit) rightOutput = speed_limit;
        if (rightOutput < -speed_limit) rightOutput = -speed_limit;
        
        // Slew rate limiting: cap how fast output can change (prevents tipping)
        if (leftOutput - lastLeftOutput > _max_accel) leftOutput = lastLeftOutput + _max_accel;
        if (leftOutput - lastLeftOutput < -_max_accel) leftOutput = lastLeftOutput - _max_accel;
        if (rightOutput - lastRightOutput > _max_accel) rightOutput = lastRightOutput + _max_accel;
        if (rightOutput - lastRightOutput < -_max_accel) rightOutput = lastRightOutput - _max_accel;
        
        lastLeftOutput = leftOutput;
        lastRightOutput = rightOutput;
        
        _left_motor.spin(forward, leftOutput, percent);
        _right_motor.spin(forward, rightOutput, percent);
        
        if (std::abs(leftError) < 5 && std::abs(rightError) < 5) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }
        vex::task::sleep(20);
    }
    _left_motor.stop();
    _right_motor.stop();
}

void Drivetrain::drive_for(double distance, double timeout, double speed_limit, double target_heading) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    int settle = 0;
    double heading_error, heading_correction, speed, current_pos, left_speed, right_speed;
    _left_motor.setPosition(0, vex::degrees);
    _right_motor.setPosition(0, vex::degrees);
    _drive_pid.reset();
    _heading_pid.reset();
    double dist = helpers::mmToDegrees(distance);
    while (timeout > 0 && bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        heading_error = helpers::angular_difference(bot::sensors::left_imu.heading(vex::degrees), target_heading);
        heading_correction = _heading_pid.compute(heading_error, 0.0, 0.02);
        current_pos = (_left_motor.position(vex::degrees) + _right_motor.position(vex::degrees)) / 2.0;
        speed = _drive_pid.compute(dist, current_pos, 0.02);
        left_speed = speed + heading_correction;
        right_speed = speed - heading_correction;
        left_speed = math::clamp(left_speed, -speed_limit, speed_limit);
        right_speed = math::clamp(right_speed, -speed_limit, speed_limit);
        left_speed *= (_max_voltage / 100.0);
        right_speed *= (_max_voltage / 100.0);
        _left_motor.spin(forward, left_speed, vex::voltageUnits::volt);
        _right_motor.spin(forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(dist - current_pos) < 25
        && std::abs(heading_error) < 1.0) {
            settle++;
        } else {
            settle = 0;
        }
        vex::task::sleep(20);
        if (settle >= 3) break;
    }
    _left_motor.spin(forward, 0, vex::voltageUnits::volt);
    _right_motor.spin(forward, 0, vex::voltageUnits::volt);
}

void Drivetrain::arcade_drive(double fwd, double turn) {
    double leftSpeed = fwd + turn;
    double rightSpeed = fwd - turn;
    double voltageScale = _max_voltage / 100.0;
    leftSpeed = std::min(std::max(leftSpeed, -100.0), 100.0);
    rightSpeed = std::min(std::max(rightSpeed, -100.0), 100.0);
    
    if (std::abs(leftSpeed) < 1.0 && std::abs(rightSpeed) < 1.0) {
        _left_motor.stop();
        _right_motor.stop();
    } else {
        _left_motor.spin(forward, leftSpeed * voltageScale, volt);
        _right_motor.spin(forward, rightSpeed * voltageScale, volt);
    }
}

void Drivetrain::tank_drive(double left, double right) {
    double voltageScale = _max_voltage / 100.0;
    left = std::min(std::max(left, -100.0), 100.0);
    right = std::min(std::max(right, -100.0), 100.0);
    
    if (std::abs(left) < 1.0 && std::abs(right) < 1.0) {
        _left_motor.stop();
        _right_motor.stop();
    } else {
        _left_motor.spin(forward, left * voltageScale, vex::voltageUnits::volt);
        _right_motor.spin(forward, right * voltageScale, vex::voltageUnits::volt);
    }
}

void Drivetrain::match_load(std::int8_t times) {
    for (std::int8_t i = 0; i < times; i++) {
        drive_for(200, 300, 50);
        vex::task::sleep(200);
    }
}

std::vector<Drivetrain::IntersectionPoint> Drivetrain::circleSegmentIntersections(
    double cx, double cy, double radius,
    double x1, double y1, double x2, double y2) {
    
    std::vector<IntersectionPoint> pts;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double fx = x1 - cx;
    double fy = y1 - cy;
    
    double A = dx * dx + dy * dy;
    double B = 2.0 * (fx * dx + fy * dy);
    double C = (fx * fx + fy * fy) - radius * radius;
    
    double disc = B * B - 4.0 * A * C;
    if (disc < 0) return pts;
    
    double discSqrt = std::sqrt(disc);
    double t1 = (-B - discSqrt) / (2.0 * A);
    double t2 = (-B + discSqrt) / (2.0 * A);
    
    if (t1 >= 0 && t1 <= 1) {
        pts.push_back({x1 + t1 * dx, y1 + t1 * dy, t1});
    }
    if (t2 >= 0 && t2 <= 1 && std::abs(t2 - t1) > 0.001) {
        pts.push_back({x1 + t2 * dx, y1 + t2 * dy, t2});
    }
    return pts;
}

void Drivetrain::handle_intake_mode(IntakeMode mode) {
    switch (mode) {
        case IntakeMode::STOP:
            bot::motors::lower.stop();
            break;
        case IntakeMode::INTAKE:
            bot::motors::lower.spin(forward, 100, percent);
            break;
        case IntakeMode::OUTTAKE:
            bot::motors::lower.spin(reverse, 100, percent);
            break;
    }

}

void Drivetrain::stop() {
    _left_motor.stop();
    _right_motor.stop();
}

void Drivetrain::brake() {
    _left_motor.setStopping(vex::brakeType::brake);
    _right_motor.setStopping(vex::brakeType::brake);
}

void Drivetrain::coast() {
    _left_motor.setStopping(vex::brakeType::coast);
    _right_motor.setStopping(vex::brakeType::coast);
}

void Drivetrain::follow_path(std::vector<Waypoint>& path, Location& location,
    double lookahead, double maxSpeed, double stopDist, double timeout) {
    
    // Tuning parameters (use member variables)
    const double kHeading = _pp_k_heading;
    const double kSlow = _pp_k_slow;
    const double minSpeed = _pp_min_speed_pct * maxSpeed;
    const double maxCurvSpeedFactor = _pp_max_curv_speed_factor;
    const double dt = PP_DT;                // Loop timestep
    
    if (path.size() < 2) {
        _left_motor.stop();
        _right_motor.stop();
        return;
    }
    
    std::size_t lastIndex = path.size() - 1;
    std::size_t closestSegIndex = 0;
    std::size_t lastIntakeIndex = 0;
    
    // Handle first waypoint's intake mode
    handle_intake_mode(path[0].intake_mode);
    
    double start_time = bot::Brain.Timer.time(vex::msec);
    
    while (true) {
        // Check timeout condition (0 = no timeout)
        if (timeout > 0 && (bot::Brain.Timer.time(vex::msec) - start_time) >= timeout) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }
        // 1) Get current pose from odometry
        double rx = location.get_x();
        double ry = location.get_y();
        double rtheta = _imu.get_heading() * M_PI / 180.0; // Convert to radians
        
        // 2) Exit condition: close enough to goal
        Waypoint& goal = path[lastIndex];
        double distToGoal = std::hypot(goal.x - rx, goal.y - ry);
        
        if (distToGoal < stopDist) {
            _left_motor.stop();
            _right_motor.stop();
            // Handle final waypoint intake mode
            handle_intake_mode(goal.intake_mode);
            break;
        }
        
        // 3) Find lookahead point on the path
        bool lookPtFound = false;
        double lookX = goal.x;
        double lookY = goal.y;
        
        // (A) Update closest segment index
        double bestDist = 1e9;
        for (std::size_t i = 0; i < path.size() - 1; i++) {
            double d = helpers::distancePointToSegment(rx, ry, 
                path[i].x, path[i].y, path[i+1].x, path[i+1].y);
            if (d < bestDist) {
                bestDist = d;
                closestSegIndex = i;
            }
        }
        
        // Handle intake mode for waypoints we've passed
        for (std::size_t i = lastIntakeIndex + 1; i <= closestSegIndex + 1 && i < path.size(); i++) {
            handle_intake_mode(path[i].intake_mode);
            lastIntakeIndex = i;
        }
        
        // (B) Find first intersection of circle with path segments
        for (std::size_t i = closestSegIndex; i < path.size() - 1; i++) {
            auto pts = circleSegmentIntersections(
                rx, ry, lookahead,
                path[i].x, path[i].y, path[i+1].x, path[i+1].y
            );
            
            if (!pts.empty()) {
                // Choose intersection farthest along segment
                IntersectionPoint best = pts[0];
                for (const auto& p : pts) {
                    if (p.t > best.t) best = p;
                }
                lookX = best.x;
                lookY = best.y;
                lookPtFound = true;
                break;
            }
        }
        
        // Fallback if no intersection found
        if (!lookPtFound) {
            for (std::size_t i = closestSegIndex; i <= lastIndex; i++) {
                if (std::hypot(path[i].x - rx, path[i].y - ry) >= lookahead) {
                    lookX = path[i].x;
                    lookY = path[i].y;
                    break;
                }
            }
        }
        
        // 4) Transform lookahead point into robot frame
        double dx = lookX - rx;
        double dy = lookY - ry;
        double localX =  std::cos(rtheta) * dx + std::sin(rtheta) * dy;
        double localY = -std::sin(rtheta) * dx + std::cos(rtheta) * dy;
        
        // 5) Compute curvature (Pure Pursuit)
        double L = std::max(lookahead, 1e-6);
        double curvature = -(2.0 * localY) / (L * L);
        
        // 6) Choose linear speed
        double endFactor = std::min(std::max(distToGoal / (lookahead * 2.0), 0.0), 1.0);
        double turnFactor = std::min(std::max(1.0 - std::abs(curvature) * _track_width, maxCurvSpeedFactor), 1.0);
        double v = maxSpeed * (kSlow + (1.0 - kSlow) * endFactor) * turnFactor;
        v = std::min(std::max(v, minSpeed), maxSpeed);
        
        // Optional heading correction
        double targetHeading = std::atan2(lookY - ry, lookX - rx);
        double headingErr = helpers::wrapToPi(targetHeading - rtheta);
        double omegaHeading = kHeading * headingErr;
        
        // Combined angular velocity
        double omega = v * curvature + omegaHeading;
        
        // 7) Convert to wheel speeds
        double vLeft  = v - omega * (_track_width * 0.5);
        double vRight = v + omega * (_track_width * 0.5);
        
        // 8) Clamp to motor range
        vLeft  = std::min(std::max(vLeft,  -maxSpeed), maxSpeed);
        vRight = std::min(std::max(vRight, -maxSpeed), maxSpeed);
        
        // 9) Send to drivetrain
        _left_motor.spin(forward, vLeft, percent);
        _right_motor.spin(forward, vRight, percent);
        
        vex::task::sleep(static_cast<int>(dt * 1000));
    }
}

void Drivetrain::follow_path_reverse(std::vector<Waypoint>& path, Location& location,
    double lookahead, double maxSpeed, double stopDist, double timeout) {
    
    // Tuning parameters (use member variables)
    const double kHeading = _pp_k_heading;
    const double kSlow = _pp_k_slow;
    const double minSpeed = _pp_min_speed_pct * maxSpeed;
    const double maxCurvSpeedFactor = _pp_max_curv_speed_factor;
    const double dt = PP_DT;                // Loop timestep
    
    if (path.size() < 2) {
        _left_motor.stop();
        _right_motor.stop();
        return;
    }
    
    std::size_t lastIndex = path.size() - 1;
    std::size_t closestSegIndex = 0;
    std::size_t lastIntakeIndex = 0;
    
    // Handle first waypoint's intake mode
    handle_intake_mode(path[0].intake_mode);
    
    double start_time = bot::Brain.Timer.time(vex::msec);
    
    while (true) {
        // Check timeout condition (0 = no timeout)
        if (timeout > 0 && (bot::Brain.Timer.time(vex::msec) - start_time) >= timeout) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }
        // 1) Get current pose from odometry
        double rx = location.get_x();
        double ry = location.get_y();
        double rtheta = _imu.get_heading() * M_PI / 180.0; // Convert to radians
        
        // 2) Exit condition: close enough to goal
        Waypoint& goal = path[lastIndex];
        double distToGoal = std::hypot(goal.x - rx, goal.y - ry);
        
        if (distToGoal < stopDist) {
            _left_motor.stop();
            _right_motor.stop();
            // Handle final waypoint intake mode
            handle_intake_mode(goal.intake_mode);
            break;
        }
        
        // 3) Find lookahead point on the path
        bool lookPtFound = false;
        double lookX = goal.x;
        double lookY = goal.y;
        
        // (A) Update closest segment index
        double bestDist = 1e9;
        for (std::size_t i = 0; i < path.size() - 1; i++) {
            double d = helpers::distancePointToSegment(rx, ry, 
                path[i].x, path[i].y, path[i+1].x, path[i+1].y);
            if (d < bestDist) {
                bestDist = d;
                closestSegIndex = i;
            }
        }
        
        // Handle intake mode for waypoints we've passed
        for (std::size_t i = lastIntakeIndex + 1; i <= closestSegIndex + 1 && i < path.size(); i++) {
            handle_intake_mode(path[i].intake_mode);
            lastIntakeIndex = i;
        }
        
        // (B) Find first intersection of circle with path segments
        for (std::size_t i = closestSegIndex; i < path.size() - 1; i++) {
            auto pts = circleSegmentIntersections(
                rx, ry, lookahead,
                path[i].x, path[i].y, path[i+1].x, path[i+1].y
            );
            
            if (!pts.empty()) {
                // Choose intersection farthest along segment
                IntersectionPoint best = pts[0];
                for (const auto& p : pts) {
                    if (p.t > best.t) best = p;
                }
                lookX = best.x;
                lookY = best.y;
                lookPtFound = true;
                break;
            }
        }
        
        // Fallback if no intersection found
        if (!lookPtFound) {
            for (std::size_t i = closestSegIndex; i <= lastIndex; i++) {
                if (std::hypot(path[i].x - rx, path[i].y - ry) >= lookahead) {
                    lookX = path[i].x;
                    lookY = path[i].y;
                    break;
                }
            }
        }
        
        // 4) Transform lookahead point into robot frame (same as forward)
        double dx = lookX - rx;
        double dy = lookY - ry;
        double localX =  std::cos(rtheta) * dx + std::sin(rtheta) * dy;
        double localY = -std::sin(rtheta) * dx + std::cos(rtheta) * dy;
        
        // 5) Compute curvature (Pure Pursuit) - same as forward, v is negative
        double L = std::max(lookahead, 1e-6);
        double curvature = (2.0 * localY) / (L * L);
        
        // 6) Choose linear speed (negative for reverse)
        double endFactor = std::min(std::max(distToGoal / (lookahead * 2.0), 0.0), 1.0);
        double turnFactor = std::min(std::max(1.0 - std::abs(curvature) * _track_width, maxCurvSpeedFactor), 1.0);
        double v = -maxSpeed * (kSlow + (1.0 - kSlow) * endFactor) * turnFactor;
        v = std::max(std::min(v, -minSpeed), -maxSpeed);
        
        // Heading correction - compute for back to point at target
        // When reversing, steering is inverted, so negate the heading correction
        double targetHeading = std::atan2(lookY - ry, lookX - rx);
        double backTheta = rtheta + M_PI;
        double headingErr = helpers::wrapToPi(targetHeading - backTheta);
        double omegaHeading = -kHeading * headingErr;
        
        // Combined angular velocity
        double omega = v * curvature + omegaHeading;
        
        // 7) Convert to wheel speeds
        double vLeft  = v - omega * (_track_width * 0.5);
        double vRight = v + omega * (_track_width * 0.5);
        
        // 8) Clamp to motor range
        vLeft  = std::min(std::max(vLeft,  -maxSpeed), maxSpeed);
        vRight = std::min(std::max(vRight, -maxSpeed), maxSpeed);
        
        // 9) Send to drivetrain
        _left_motor.spin(forward, vLeft, percent);
        _right_motor.spin(forward, vRight, percent);
        
        vex::task::sleep(static_cast<int>(dt * 1000));
    }
}
/*
void Drivetrain::drive_to(double x, double y, double timeout, double speed_limit) {
    // Get current position from global MCL location
    double startX = static_cast<double>(bot::mcl::location.get_x());
    double startY = static_cast<double>(bot::mcl::location.get_y());
    double endX = static_cast<double>(x);
    double endY = static_cast<double>(y);
    double dist = std::hypot(endX - startX, endY - startY);
    double angle = std::atan2(endY - startY, endX - startX);
    angle = angle * 180.0 / M_PI;
    //angle = 180.0-angle;
    while (angle < -180.0) angle += 360;
    while (angle > 180.0) angle -= 360;
    turn_to_heading(angle, 1000, speed_limit);
    drive_for(dist, timeout- 1000, speed_limit);
}

void Drivetrain::drive_to_reverse(double x, double y, double timeout, double speed_limit) {
    double startX = static_cast<double>(bot::mcl::location.get_x());
    double startY = static_cast<double>(bot::mcl::location.get_y());
    double endX = static_cast<double>(x);
    double endY = static_cast<double>(y);
    double dist = std::hypot(endX - startX, endY - startY);
    double angle = std::atan2(endY - startY, endX - startX);

    angle = angle * 180.0 / M_PI;
    angle = 180.0-angle;
    while (angle < -180.0) angle += 360;
    while (angle > 180.0) angle -= 360;
    angle *= -1;
    turn_to_heading(angle, 1000, speed_limit);
    drive_for(-dist, timeout- 1000, speed_limit);
}
*/

void Drivetrain::drive_to(double x, double y, double stop_dist, double timeout, double speed_limit) {


    /*double kHeading = _pp_k_heading;
    double kSlow = _pp_k_slow;
    double heading_threshold = 1e-2;

    double start_time = bot::Brain.Timer.time(vex::msec);

    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {

        
        double current_x = location.get_x();
        double current_y = location.get_y();
        double current_heading = static_cast<double>(location.get_heading());
        
        double dx = x - current_x;
        double dy = y - current_y;
        double dist = std::hypot(dx, dy);

        double slowdown = dist * kSlow;
        slowdown = math::clamp(slowdown, 0.0, 1.0);

        if (dist < stop_dist) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }

        double tx = dx/dist;
        double ty = dy/dist;

        if (prev_poses.size() < 1) {
            prev_poses.push_back({current_x, current_y, current_heading});
            continue;
        }
        double prev_x = prev_poses[0].x;
        double prev_y = prev_poses[0].y;
        double vx = current_x - prev_x;
        double vy = current_y - prev_y;
        double vmag = std::hypot(vx, vy);
        
        double rx, ry;

        if (vmag > heading_threshold) {
            rx = vx/vmag;
            ry = vy/vmag;
        } else {
            rx = tx;
            ry = ty;
        }

        double error = rx * ty - ry * tx;



        double left_output = 0.67 * speed_limit - kHeading * error;
        double right_output = 0.67 * speed_limit + kHeading * error;

        left_output *= slowdown;
        right_output *= slowdown;

        left_output = math::clamp(left_output, -speed_limit, speed_limit);
        right_output = math::clamp(right_output, -speed_limit, speed_limit);
        printf("dist: %.2f\n", dist);
        printf("Error: %.2f\n", error);
        printf("Left Output: %.2f, Right Output: %.2f\n", left_output, right_output);
        tank_drive(left_output, right_output);
        vex::task::sleep(static_cast<int>(PP_DT * 1000));
    }
    _left_motor.stop();
    _right_motor.stop();
    return;*/
}

void Drivetrain::drive_to_reverse(double x, double y, double stop_dist, double timeout, double speed_limit){
    std::vector<Pose> prev_poses = {};

    prev_poses.push_back({
        static_cast<double>(bot::mcl::location.get_x()),
        static_cast<double>(bot::mcl::location.get_y()),
        static_cast<double>(bot::mcl::location.get_heading())
    });

    double kHeading = _pp_k_heading;
    double kSlow = _pp_k_slow;
    double heading_threshold = 1e-2;

    double start_time = bot::Brain.Timer.time(vex::msec);

    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        double current_x = bot::mcl::location.get_x();
        double current_y = bot::mcl::location.get_y();
        double current_heading = static_cast<double>(bot::mcl::location.get_heading());
        prev_poses.push_back({current_x, current_y, current_heading});
        if (prev_poses.size() > 5) prev_poses.erase(prev_poses.begin());

        double dx = x - current_x;
        double dy = y - current_y;
        double dist = std::hypot(dx, dy);

        if (dist < stop_dist) {
            _left_motor.stop();
            _right_motor.stop();
            return;
        }

        double tx = dx/dist;
        double ty = dy/dist;
        
        tx *= -1;
        ty *= -1; 

        double vx = current_x - prev_poses[0].x;
        double vy = current_y - prev_poses[0].y;
        double vmag = std::hypot(vx, vy);
        
        double rx, ry;

        if (vmag > heading_threshold) {
            rx = vx/vmag;
            ry = vy/vmag;
        } else {
            rx = tx;
            ry = ty;
        }

        double error = rx * ty - ry * tx;

        double slowdown = dist * kSlow;
        slowdown = math::clamp(slowdown, 0.0, 1.0);

        double left_output = 0.67 * speed_limit - kHeading * error;
        double right_output = 0.67 * speed_limit + kHeading * error;

        left_output *= slowdown;
        right_output *= slowdown;

        left_output = math::clamp(left_output, -speed_limit, speed_limit);
        right_output = math::clamp(right_output, -speed_limit, speed_limit);
        
        tank_drive(-left_output, -right_output);
        vex::task::sleep(static_cast<int>(PP_DT * 1000));
    }

    _left_motor.stop();
    _right_motor.stop();
    return;
}

double Drivetrain::get_left_encoder() {
    return _left_motor.position(vex::degrees);
}

double Drivetrain::get_right_encoder() {
    return _right_motor.position(vex::degrees);
}

void Drivetrain::set_pp_k_heading(double k_heading) {
    _pp_k_heading = k_heading;
}

void Drivetrain::set_pp_k_slow(double k_slow) {
    _pp_k_slow = k_slow;
}

void Drivetrain::set_pp_max_curv_speed_factor(double factor) {
    _pp_max_curv_speed_factor = factor;
}

void Drivetrain::set_pp_min_speed_pct(double pct) {
    _pp_min_speed_pct = pct;
}