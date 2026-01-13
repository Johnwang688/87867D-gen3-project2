#include "drivetrain.hpp"

Drivetrain::Drivetrain(vex::motor_group& left_motor, vex::motor_group& right_motor,
    bot::sensors::inertial_group& imu) : 
    _left_motor(left_motor), _right_motor(right_motor),
    _wheel_diameter(WHEEL_DIAMETER), _track_width(TRACK_WIDTH), _gear_ratio(GEAR_RATIO),
    _drive_kp(DRIVE_KP), _drive_ki(DRIVE_KI), _drive_kd(DRIVE_KD),
    _turn_kp(TURN_KP), _turn_ki(TURN_KI), _turn_kd(TURN_KD),
    _max_voltage(MAX_VOLTAGE),
    _max_accel(MAX_ACCEL),
    _imu(imu),
    _left_drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _right_drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _left_turn_pid(TURN_KP, TURN_KI, TURN_KD),
    _right_turn_pid(TURN_KP, TURN_KI, TURN_KD),
    _left_arc_pid(ARC_KP, ARC_KI, ARC_KD),
    _right_arc_pid(ARC_KP, ARC_KI, ARC_KD)
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
    
    double start_time = bot::Brain.Timer.time(vex::msec);
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        double leftCurrentPos = _left_motor.position(degrees);
        double rightCurrentPos = _right_motor.position(degrees);
        
        double leftError = leftTargetPos - leftCurrentPos;
        double rightError = rightTargetPos - rightCurrentPos;
        
        double leftOutput = _left_drive_pid.compute(leftTargetPos, leftCurrentPos, 0.02);
        double rightOutput = _right_drive_pid.compute(rightTargetPos, rightCurrentPos, 0.02);
        
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
    double lastOutput = 0.0;
    double start_time = bot::Brain.Timer.time(vex::msec);

    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        double currentHeading = _imu.get_heading();

        double error = helpers::angular_difference(currentHeading, targetHeading);
        
        // trick: (double setpoint = error, double input = 0.0) sets error in PID automatically to error.
        double output = _left_turn_pid.compute(error, 0.0, 0.02);

        output = math::clamp(output, -speed_limit, speed_limit);
        
        //cap how fast output can change (prevents tipping)
        if (output - lastOutput > _max_accel) output = lastOutput + _max_accel;
        if (output - lastOutput < -_max_accel) output = lastOutput - _max_accel;
        lastOutput = output;

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
        double leftCurrentPos = _left_motor.position(degrees);
        double rightCurrentPos = _right_motor.position(degrees);
        
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

void Drivetrain::follow_path(std::vector<Waypoint>& path, Location& location,
    double lookahead, double maxSpeed, double stopDist) {
    
    // Tuning parameters
    const double kHeading = 2.0;           // Heading correction gain
    const double kSlow = 0.5;              // Slowdown factor near end
    const double minSpeed = 0.15 * maxSpeed;
    const double maxCurvSpeedFactor = 0.6; // Slow down on sharp turns
    const double dt = 0.05;                // Loop timestep (50ms)
    
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
    
    while (true) {
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
        double curvature = (2.0 * localY) / (L * L);
        
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

double Drivetrain::get_left_encoder() {
    return _left_motor.position(vex::degrees);
}

double Drivetrain::get_right_encoder() {
    return _right_motor.position(vex::degrees);
}