#include "bot.hpp"
#include "drivetrain.hpp"
#include "location.hpp"
using namespace vex;


struct SensorConfig {
    float offX, offY, relativeAngle;
};

static const SensorConfig sensor_layout[8] = {
    {static_cast<float>(FRONT_LEFT_OFFSET_X),  static_cast<float>(FRONT_LEFT_OFFSET_Y),   0.0f},   // 0: front_left - points forward
    {static_cast<float>(FRONT_RIGHT_OFFSET_X), static_cast<float>(FRONT_RIGHT_OFFSET_Y),  0.0f},   // 1: front_right - points forward
    {static_cast<float>(BACK_LEFT_OFFSET_X),   static_cast<float>(BACK_LEFT_OFFSET_Y),   180.0f}, // 2: back_left - points backward
    {static_cast<float>(BACK_RIGHT_OFFSET_X),  static_cast<float>(BACK_RIGHT_OFFSET_Y),  180.0f}, // 3: back_right - points backward
    {static_cast<float>(LEFT_FORWARD_OFFSET_X), static_cast<float>(LEFT_FORWARD_OFFSET_Y), -90.0f}, // 4: left_forward - points left
    {static_cast<float>(LEFT_AFT_OFFSET_X),     static_cast<float>(LEFT_AFT_OFFSET_Y),     -90.0f}, // 5: left_aft - points left
    {static_cast<float>(RIGHT_FORWARD_OFFSET_X), static_cast<float>(RIGHT_FORWARD_OFFSET_Y), 90.0f}, // 6: right_forward - points right
    {static_cast<float>(RIGHT_AFT_OFFSET_X),    static_cast<float>(RIGHT_AFT_OFFSET_Y),    90.0f}  // 7: right_aft - points right
};

Location::Location(Drivetrain& drivetrain, 
    vex::distance& left_distance_forward, 
    vex::distance& right_distance_forward, 
    vex::distance& left_distance_aft, 
    vex::distance& right_distance_aft, 
    vex::distance& front_distance_left, 
    vex::distance& front_distance_right, 
    vex::distance& back_distance_left, 
    vex::distance& back_distance_right,
    bot::sensors::inertial_group& imu) : 
    _drivetrain(drivetrain), 
    _left_distance_forward(left_distance_forward), 
    _right_distance_forward(right_distance_forward), 
    _left_distance_aft(left_distance_aft), 
    _right_distance_aft(right_distance_aft), 
    _front_distance_left(front_distance_left), 
    _front_distance_right(front_distance_right), 
    _back_distance_left(back_distance_left), 
    _back_distance_right(back_distance_right), 
    _imu(imu),
    _x(0), _y(0), _heading(0),
    _last_left_encoder(0), _last_right_encoder(0),
    _updateTask(),
    _bestParticle{0, 0, 0, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, true},
    _taskRunning(false)
{
    _particles.reserve(9);
}

void Location::reset(std::int16_t x, std::int16_t y, std::int16_t heading) {
    // Store heading directly in USER convention (0° = +Y, CW positive)
    // This matches the backtest convention
    _x = x;
    _y = y;
    _heading = heading;
    
    // Reset encoder baselines to prevent drift on first update
    _last_left_encoder = _drivetrain.get_left_encoder();
    _last_right_encoder = _drivetrain.get_right_encoder();
    
    // Initialize best particle to the reset position
    _bestParticle.x = x;
    _bestParticle.y = y;
    _bestParticle.heading = heading;
    
    // Reset jump filtering state
    _consecutiveJumpCount = 0;
    for (int i = 0; i < 3; i++) {
        _jumpHistory[i] = {x, y};
    }
}

std::int16_t Location::get_x() {
    _positionMutex.lock();
    std::int16_t x = _x;
    _positionMutex.unlock();
    return x;
}

std::int16_t Location::get_y() {
    _positionMutex.lock();
    std::int16_t y = _y;
    _positionMutex.unlock();
    return y;
}

std::int16_t Location::get_heading() {
    _positionMutex.lock();
    std::int16_t heading = _heading;
    _positionMutex.unlock();
    
    // Already in USER convention (0° = +Y), no conversion needed
    return heading;
}

int Location::_updateTaskWrapper(void* instance) {
    Location* loc = static_cast<Location*>(instance);
    while (true) {
        loc->update();
        task::sleep(50);
    }
    return 0;
}

void Location::start() {
    if (!_taskRunning) {
        _updateTask = task(_updateTaskWrapper, this);
        _taskRunning = true;
    }
}

void Location::stop() {
    if (_taskRunning) {
        _updateTask.stop();
        _taskRunning = false;
    }
}

// Helper to safely read distance sensor with error handling and clamping
static inline std::uint16_t safe_distance_read(vex::distance& sensor) {
    double raw = sensor.objectDistance(mm);
    

    if (raw < 0.0 || std::isnan(raw) || raw >= 9999.0) {
        return 2000; 
    }
    
    // Clamp to 2000 max
    if (raw > 2000.0) {
        return 2000;
    }
    
    return static_cast<std::uint16_t>(raw);
}

Reading Location::capture_readings() {
    Reading readings;
    readings.front_left    = safe_distance_read(_front_distance_left);
    readings.front_right   = safe_distance_read(_front_distance_right);
    readings.back_left     = safe_distance_read(_back_distance_left);
    readings.back_right    = safe_distance_read(_back_distance_right);
    readings.left_forward  = safe_distance_read(_left_distance_forward);
    readings.left_aft      = safe_distance_read(_left_distance_aft);
    readings.right_forward = safe_distance_read(_right_distance_forward);
    readings.right_aft     = safe_distance_read(_right_distance_aft);
    return readings;
}

Particle Location::estimate() {
    // 1. Calculate the change in encoder distance since the last loop
    // Note: You'll need to store 'last_encoder_values' as private members
    double left_dist = _drivetrain.get_left_encoder();
    double right_dist = _drivetrain.get_right_encoder();
    
    double d_left = left_dist - _last_left_encoder;
    double d_right = right_dist - _last_right_encoder;
    double d_center = (d_left + d_right) / 2.0;

    // 2. Get the current heading from the IMU and convert to USER convention
    // IMU returns 90° when facing +Y, and CW (right) increases the value
    // Convert to USER convention: 0° = +Y, CW positive
    double imu_raw = _imu.get_heading();
    double current_imu_heading = imu_raw - 90.0;
    // Normalize to [-180, 180]
    if (current_imu_heading > 180.0) current_imu_heading -= 360.0;
    if (current_imu_heading <= -180.0) current_imu_heading += 360.0;
    
    // 3. Update 'last' values for the next cycle
    _last_left_encoder = left_dist;
    _last_right_encoder = right_dist;

    // 4. Calculate the local movement vector
    // Using the average heading during the move (Arc Length approximation)
    double avg_heading_deg = helpers::average_heading(_heading, current_imu_heading);
    double avg_heading_rad = math::to_rad(avg_heading_deg);
    
    // Convert encoder degrees to mm
    double d_center_mm = d_center * MM_PER_TICK;
    
    // USER convention: 0° = +Y, 90° = +X
    // Forward direction = (sin, cos), so x += d*sin(h), y += d*cos(h)
    Particle delta;
    delta.x = static_cast<int16_t>(_x + (d_center_mm * sin(avg_heading_rad)));
    delta.y = static_cast<int16_t>(_y + (d_center_mm * cos(avg_heading_rad)));
    delta.heading = static_cast<int16_t>(current_imu_heading);
    // Initialize sensor readings to 2000 (out of range) to avoid uninitialized values
    delta.front_left = delta.front_right = delta.back_left = delta.back_right = 2000;
    delta.left_forward = delta.left_aft = delta.right_forward = delta.right_aft = 2000;
    delta.valid = true;

    return delta; 
}

float Location::calculate_intersection(float s_x, float s_y, float robot_angle_deg, const Line& wall) {
    // Convert robot frame angle to direction vector in world coordinates
    float r_dx, r_dy;
    math::robot_angle_to_direction(robot_angle_deg, r_dx, r_dy);

    //printf("r_dx: %f r_dy: %f\n", r_dx, r_dy);

    float w_dx = wall.x2 - wall.x1;
    float w_dy = wall.y2 - wall.y1;

    //printf("w_dx: %f w_dy: %f\n", w_dx, w_dy);

    // Standard line-line intersection formula:
    // Ray: P = (s_x, s_y) + t * (r_dx, r_dy)
    // Wall: Q = (wall.x1, wall.y1) + u * (w_dx, w_dy)
    // At intersection: (s_x, s_y) + t*(r_dx, r_dy) = (wall.x1, wall.y1) + u*(w_dx, w_dy)
    // Rearranging: (wall.x1 - s_x, wall.y1 - s_y) = t*(r_dx, r_dy) - u*(w_dx, w_dy)
    // 
    // Using standard formula: denominator = r_dx * w_dy - r_dy * w_dx
    float denominator = (r_dx * w_dy) - (r_dy * w_dx);

    if (std::abs(denominator) < 0.0001f) return 2000.0f;

    float t = ((wall.x1 - s_x) * w_dy - (wall.y1 - s_y) * w_dx) / denominator;
    float u = ((wall.x1 - s_x) * r_dy - (wall.y1 - s_y) * r_dx) / denominator;

    if (t > 0 && u >= 0 && u <= 1) {
        return t;
    }

    return 2000.0f;
}

/*float Location::calculate_intersection(float s_x, float s_y, float angle_deg, const Line& wall) {
    float rad = math::to_rad(angle_deg);
    float r_dx = std::cos(rad);
    float r_dy = std::sin(rad);

    float w_dx = wall.x2 - wall.x1;
    float w_dy = wall.y2 - wall.y1;

    float denominator = (w_dx * r_dy) - (w_dy * r_dx);

    if (std::abs(denominator) < 0.0001f) return 2000.0f;

    float t = ((wall.x1 - s_x) * w_dy - (wall.y1 - s_y) * w_dx) / denominator;
    float u = ((wall.x1 - s_x) * r_dy - (wall.y1 - s_y) * r_dx) / denominator;

    if (t > 0 && u >= 0 && u <= 1) {
        return t;
    }

    return 2000.0f;
}*/

bool Location::is_particle_valid(const Particle& particle) {
    // Check if particle center is within bounds
    if (particle.x < -OUT_OF_BOUNDS || particle.x > OUT_OF_BOUNDS || 
        particle.y < -OUT_OF_BOUNDS || particle.y > OUT_OF_BOUNDS) {
        return false;
    }
    
    // Check if all sensor positions are within field bounds
    // particle.heading is in USER convention: 0° = +Y, 90° = +X
    float rad_h = math::to_rad(particle.heading);
    float cos_h = std::cos(rad_h);
    float sin_h = std::sin(rad_h);
    
    for (int i = 0; i < 8; ++i) {
        // Robot frame: offX = right, offY = forward
        // USER convention: forward = (sin_h, cos_h), right = (cos_h, -sin_h)
        float s_x = particle.x + (sensor_layout[i].offY * sin_h + sensor_layout[i].offX * cos_h);
        float s_y = particle.y + (sensor_layout[i].offY * cos_h - sensor_layout[i].offX * sin_h);
        
        if (s_x < -OUT_OF_BOUNDS || s_x > OUT_OF_BOUNDS || 
            s_y < -OUT_OF_BOUNDS || s_y > OUT_OF_BOUNDS) {
            return false;  // Sensor is outside field bounds
        }
    }
    
    return true;
}

void Location::raycast(Particle& particle) {
    uint16_t* p_readings[8] = {
        &particle.front_left, &particle.front_right, &particle.back_left, &particle.back_right,
        &particle.left_forward, &particle.left_aft, &particle.right_forward, &particle.right_aft
    };

    // particle.heading is in USER convention: 0° = +Y, 90° = +X
    float rad_h = math::to_rad(particle.heading);
    float cos_h = std::cos(rad_h);
    float sin_h = std::sin(rad_h);

    for (int i = 0; i < 8; ++i) {
        // Transform sensor position from robot frame to world frame
        // Robot frame: offX = right, offY = forward
        // USER convention: forward = (sin_h, cos_h), right = (cos_h, -sin_h)
        float s_x = particle.x + (sensor_layout[i].offY * sin_h + sensor_layout[i].offX * cos_h);
        float s_y = particle.y + (sensor_layout[i].offY * cos_h - sensor_layout[i].offX * sin_h);

        // Sensor angle is relative to robot forward direction (robot frame)
        // Both heading and sensor angles use USER convention (0° = +Y, CW positive)
        float sensor_angle = particle.heading + sensor_layout[i].relativeAngle;

        float min_dist = 2000.0f;
        for (size_t w = 0; w < sizeof(map) / sizeof(map[0]); ++w) {
            const auto& wall = map[w];
            float d = calculate_intersection(s_x, s_y, sensor_angle, wall);
            if (d < min_dist) min_dist = d;
        }

        // Safety clamp: ensure values never exceed 2000mm
        if (min_dist > 2000.0f) min_dist = 2000.0f;
        if (min_dist < 0.0f) min_dist = 2000.0f;
        *p_readings[i] = static_cast<uint16_t>(min_dist);
    }
}

/*void Location::raycast(Particle& particle) {
    uint16_t* p_readings[8] = {
        &particle.front_left, &particle.front_right, &particle.back_left, &particle.back_right,
        &particle.left_forward, &particle.left_aft, &particle.right_forward, &particle.right_aft
    };

    float rad_h = math::to_rad(particle.heading);
    float cos_h = std::cos(rad_h);
    float sin_h = std::sin(rad_h);

    for (int i = 0; i < 8; ++i) {
        // Robot frame: offX = right, offY = forward
        // Field frame: forward = (cos_h, sin_h), right = (sin_h, -cos_h)
        float s_x = particle.x + (sensor_layout[i].offY * cos_h + sensor_layout[i].offX * sin_h);
        float s_y = particle.y + (sensor_layout[i].offY * sin_h - sensor_layout[i].offX * cos_h);

        float world_angle = particle.heading + sensor_layout[i].relativeAngle;

        float min_dist = 2000.0f;
        for (const auto& wall : map) {
            float d = calculate_intersection(s_x, s_y, world_angle, wall);
            if (d < min_dist) min_dist = d;
        }

        *p_readings[i] = static_cast<uint16_t>(min_dist);
    }
}*/

void Location::update() {
    Particle guess = estimate();
    Reading readings = capture_readings();
    search(guess, readings);
    
    // Calculate distance from current position to proposed new position
    double jumpDist = std::hypot(
        static_cast<double>(_bestParticle.x - _x),
        static_cast<double>(_bestParticle.y - _y)
    );
    
    bool acceptUpdate = true;
    
    if (jumpDist > JUMP_THRESHOLD) {
        // Large jump detected - check if it's consistent with recent jump candidates
        
        // Shift history and add new candidate
        _jumpHistory[2] = _jumpHistory[1];
        _jumpHistory[1] = _jumpHistory[0];
        _jumpHistory[0] = {_bestParticle.x, _bestParticle.y};
        _consecutiveJumpCount++;
        
        if (_consecutiveJumpCount >= REQUIRED_CONSECUTIVE_JUMPS) {
            // Check if all 3 jump candidates form a tight cluster
            double dist01 = std::hypot(
                static_cast<double>(_jumpHistory[0].x - _jumpHistory[1].x),
                static_cast<double>(_jumpHistory[0].y - _jumpHistory[1].y)
            );
            double dist02 = std::hypot(
                static_cast<double>(_jumpHistory[0].x - _jumpHistory[2].x),
                static_cast<double>(_jumpHistory[0].y - _jumpHistory[2].y)
            );
            double dist12 = std::hypot(
                static_cast<double>(_jumpHistory[1].x - _jumpHistory[2].x),
                static_cast<double>(_jumpHistory[1].y - _jumpHistory[2].y)
            );
            
            // All three candidates must be within CLUSTER_RADIUS of each other
            if (dist01 <= CLUSTER_RADIUS && dist02 <= CLUSTER_RADIUS && dist12 <= CLUSTER_RADIUS) {
                // Consistent cluster - accept the jump and reset counter
                _consecutiveJumpCount = 0;
                acceptUpdate = true;
            } else {
                // Not a consistent cluster - reject and keep waiting
                acceptUpdate = false;
            }
        } else {
            // Not enough consecutive jumps yet - reject this update
            acceptUpdate = false;
        }
    } else {
        // Normal small movement - reset jump counter
        _consecutiveJumpCount = 0;
    }
    
    // Lock mutex when updating shared position data
    _positionMutex.lock();
    if (acceptUpdate) {
        // Use particle filter result
        _x = _bestParticle.x;
        _y = _bestParticle.y;
        _heading = _bestParticle.heading;
    } else {
        // Jump rejected - use encoder guess as fallback
        _x = guess.x;
        _y = guess.y;
        _heading = guess.heading;
    }
    _positionMutex.unlock();
}

void Location::search(Particle guess, Reading readings) {
    // Raycast the initial guess to get proper sensor readings
    Particle bestInLayer = guess;
    bestInLayer.valid = true;
    if (is_particle_valid(bestInLayer)) {
        raycast(bestInLayer);
    }

    for (int layer = 0; layer < LAYERS; ++layer) {  
        int16_t step = INITIAL_STEP_SIZE >> layer; 
        _particles.clear();

        // Add park zone particles in first layer (encoder guess is inaccurate in park zones)
        if (layer == 0) {
            Particle parkRed = bestInLayer;
            parkRed.x = RED_PARK_ZONE_X;
            parkRed.y = RED_PARK_ZONE_Y;
            if (is_particle_valid(parkRed)) {
                parkRed.valid = true;
                raycast(parkRed);
                _particles.push_back(parkRed);
            }
            
            Particle parkBlue = bestInLayer;
            parkBlue.x = BLUE_PARK_ZONE_X;
            parkBlue.y = BLUE_PARK_ZONE_Y;
            if (is_particle_valid(parkBlue)) {
                parkBlue.valid = true;
                raycast(parkBlue);
                _particles.push_back(parkBlue);
            }
        }

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                Particle p = bestInLayer;
                p.x += dx * step;
                p.y += dy * step;
                
                if (!is_particle_valid(p)) {
                    p.valid = false;
                } else {
                    p.valid = true;
                    raycast(p);
                }
                _particles.push_back(p);
            }
        }

        uint32_t minError = 0xFFFFFFFF; 
        Particle bestCandidate = bestInLayer;
        
        // TOP-3 TRACKING: Track best 3 particles for weighted average
        struct TopParticle { Particle p; uint32_t error; };
        TopParticle top3[3] = {
            {bestInLayer, 0xFFFFFFFF},
            {bestInLayer, 0xFFFFFFFF},
            {bestInLayer, 0xFFFFFFFF}
        };
        
        // LAYER-AWARE DEADBAND: Reduce for final layers to improve precision
        bool is_precision_layer = (layer >= LAYERS - 3);
        
        for (const auto& p : _particles) {
            if (!p.valid) continue;
            
            uint32_t currentError = 0;
            uint32_t validSensorCount = 0;

            // Use GUESS (encoder estimate) for quadrant filtering, not particle position
            // This prevents feedback loops where wrong-side particles ignore damning sensors
            // All particles trust the same sensors based on encoder estimate
            auto is_sensor_facing_visible_wall = [&](float sensor_rel_angle) -> bool {
                // Both heading and sensor angles use USER convention (CW positive), so ADD
                float world_angle = guess.heading + sensor_rel_angle;
                while (world_angle > 180.0f) world_angle -= 360.0f;
                while (world_angle <= -180.0f) world_angle += 360.0f;
                
                // USER/ROBOT convention: 0° = +Y (forward), 90° = +X (right)
                // [-45, 45) → sensor points toward +Y → sees +Y wall if y >= 0
                if (world_angle >= -45.0f && world_angle < 45.0f) return guess.y >= 0;
                // [45, 135) → sensor points toward +X → sees +X wall if x >= 0
                else if (world_angle >= 45.0f && world_angle < 135.0f) return guess.x >= 0;
                // [135, 180] or [-180, -135) → sensor points toward -Y → sees -Y wall if y < 0
                else if (world_angle >= 135.0f || world_angle < -135.0f) return guess.y < 0;
                // [-135, -45) → sensor points toward -X → sees -X wall if x < 0
                else return guess.x < 0;
            };
            
            bool use_front = is_sensor_facing_visible_wall(0.0f);
            bool use_back = is_sensor_facing_visible_wall(180.0f);
            bool use_left = is_sensor_facing_visible_wall(-90.0f);
            bool use_right = is_sensor_facing_visible_wall(90.0f);

            // DYNAMIC NOISE DEADBAND: Scales with distance
            auto calc_deadband = [&](uint16_t distance) -> int {
                int base_deadband = is_precision_layer ? 10 : 30;
                constexpr int SCALE_THRESHOLD = 600;
                int scale_rate = is_precision_layer ? 30 : 15;
                
                if (distance >= 2000) return base_deadband;
                if (distance <= SCALE_THRESHOLD) return base_deadband;
                
                return base_deadband + (distance - SCALE_THRESHOLD) / scale_rate;
            };
            
            // Helper to calculate raw diff for a sensor pair
            auto calc_raw_diff = [](uint16_t sim, uint16_t act) -> int {
                if (sim >= 2000 && act >= 2000) return -1;  // Both OOR
                if (sim >= 2000 || act >= 2000) return 800; // Mismatch OOR
                return std::abs(static_cast<int>(sim) - static_cast<int>(act));
            };
            
            // Calculate raw diffs for all sensors
            int diff_front_left    = calc_raw_diff(p.front_left, readings.front_left);
            int diff_front_right   = calc_raw_diff(p.front_right, readings.front_right);
            int diff_back_left     = calc_raw_diff(p.back_left, readings.back_left);
            int diff_back_right    = calc_raw_diff(p.back_right, readings.back_right);
            int diff_left_forward  = calc_raw_diff(p.left_forward, readings.left_forward);
            int diff_left_aft      = calc_raw_diff(p.left_aft, readings.left_aft);
            int diff_right_forward = calc_raw_diff(p.right_forward, readings.right_forward);
            int diff_right_aft     = calc_raw_diff(p.right_aft, readings.right_aft);
            
            // Sensor weights (1.0 = use, 0.0 = ignore)
            float w_front_left = 1.0f, w_front_right = 1.0f;
            float w_back_left = 1.0f, w_back_right = 1.0f;
            float w_left_forward = 1.0f, w_left_aft = 1.0f;
            float w_right_forward = 1.0f, w_right_aft = 1.0f;
            
            // SENSOR PAIR CONSISTENCY CHECK
            auto check_pair = [](int diff1, int diff2, float& w1, float& w2) {
                if (diff1 < 0 || diff2 < 0 || diff1 >= 800 || diff2 >= 800) return;
                int sensor_diff = std::abs(diff1 - diff2);
                if (sensor_diff > 150) {
                    if (diff1 > diff2) { w1 = 0.0f; w2 = 1.5f; }
                    else { w2 = 0.0f; w1 = 1.5f; }
                }
            };
            
            check_pair(diff_front_left, diff_front_right, w_front_left, w_front_right);
            check_pair(diff_back_left, diff_back_right, w_back_left, w_back_right);
            check_pair(diff_left_forward, diff_left_aft, w_left_forward, w_left_aft);
            check_pair(diff_right_forward, diff_right_aft, w_right_forward, w_right_aft);
            
            // Weighted error function with dynamic deadband
            auto get_weighted_err = [&](int diff, uint16_t actual_reading, bool use_sensor, float sensor_weight) -> uint32_t {
                if (diff < 0) return 0;  // Both OOR
                if (sensor_weight <= 0.0f) return 0;  // Discarded by pair check
                
                if (diff >= 800) {  // OOR mismatch
                    validSensorCount++;
                    float base = use_sensor ? 800.0f : 160.0f;
                    return static_cast<uint32_t>(base * sensor_weight);
                }
                
                int deadband = calc_deadband(actual_reading);
                int adjusted_diff = std::max(0, diff - deadband);
                
                if (!use_sensor && adjusted_diff > 500) return 0;
                
                validSensorCount++;
                float base_weight = use_sensor ? 1.0f : 0.2f;
                return static_cast<uint32_t>(adjusted_diff * base_weight * sensor_weight);
            };

            currentError += get_weighted_err(diff_front_left,    readings.front_left,    use_front, w_front_left);
            currentError += get_weighted_err(diff_front_right,   readings.front_right,   use_front, w_front_right);
            currentError += get_weighted_err(diff_back_left,     readings.back_left,     use_back,  w_back_left);
            currentError += get_weighted_err(diff_back_right,    readings.back_right,    use_back,  w_back_right);
            currentError += get_weighted_err(diff_left_forward,  readings.left_forward,  use_left,  w_left_forward);
            currentError += get_weighted_err(diff_left_aft,      readings.left_aft,      use_left,  w_left_aft);
            currentError += get_weighted_err(diff_right_forward, readings.right_forward, use_right, w_right_forward);
            currentError += get_weighted_err(diff_right_aft,     readings.right_aft,     use_right, w_right_aft);
            
            if (validSensorCount < 2) {
                currentError = 0xFFFFFFFF;
            }

            if (currentError < minError) {
                minError = currentError;
                bestCandidate = p;
            }
            
            // UPDATE TOP-3 TRACKING
            if (currentError < top3[2].error) {
                top3[2] = {p, currentError};
                if (top3[2].error < top3[1].error) {
                    std::swap(top3[1], top3[2]);
                    if (top3[1].error < top3[0].error) {
                        std::swap(top3[0], top3[1]);
                    }
                }
            }
        }
        
        // GRADIENT SEARCH: Try moving further in the direction of improvement
        if (minError < 0xFFFFFFFF && (bestCandidate.x != bestInLayer.x || bestCandidate.y != bestInLayer.y)) {
            int16_t dx = bestCandidate.x - bestInLayer.x;
            int16_t dy = bestCandidate.y - bestInLayer.y;
            
            Particle gradient = bestCandidate;
            gradient.x += dx / 2;
            gradient.y += dy / 2;
            
            if (is_particle_valid(gradient)) {
                gradient.valid = true;
                raycast(gradient);
                
                uint32_t gradientError = 0;
                uint32_t gradientValidCount = 0;
                
                // Use GUESS for quadrant filtering (consistent with main loop)
                // USER/ROBOT convention: 0° = +Y (forward), 90° = +X (right)
                auto is_sensor_facing_visible_wall_grad = [&](float sensor_rel_angle) -> bool {
                    float world_angle = guess.heading + sensor_rel_angle;
                    while (world_angle > 180.0f) world_angle -= 360.0f;
                    while (world_angle <= -180.0f) world_angle += 360.0f;
                    if (world_angle >= -45.0f && world_angle < 45.0f) return guess.y >= 0;
                    else if (world_angle >= 45.0f && world_angle < 135.0f) return guess.x >= 0;
                    else if (world_angle >= 135.0f || world_angle < -135.0f) return guess.y < 0;
                    else return guess.x < 0;
                };
                
                bool g_front = is_sensor_facing_visible_wall_grad(0.0f);
                bool g_back = is_sensor_facing_visible_wall_grad(180.0f);
                bool g_left = is_sensor_facing_visible_wall_grad(-90.0f);
                bool g_right = is_sensor_facing_visible_wall_grad(90.0f);
                
                auto calc_raw_diff_g = [](uint16_t sim, uint16_t act) -> int {
                    if (sim >= 2000 && act >= 2000) return -1;
                    if (sim >= 2000 || act >= 2000) return 800;
                    return std::abs(static_cast<int>(sim) - static_cast<int>(act));
                };
                
                int gd_fl = calc_raw_diff_g(gradient.front_left, readings.front_left);
                int gd_fr = calc_raw_diff_g(gradient.front_right, readings.front_right);
                int gd_bl = calc_raw_diff_g(gradient.back_left, readings.back_left);
                int gd_br = calc_raw_diff_g(gradient.back_right, readings.back_right);
                int gd_lf = calc_raw_diff_g(gradient.left_forward, readings.left_forward);
                int gd_la = calc_raw_diff_g(gradient.left_aft, readings.left_aft);
                int gd_rf = calc_raw_diff_g(gradient.right_forward, readings.right_forward);
                int gd_ra = calc_raw_diff_g(gradient.right_aft, readings.right_aft);
                
                float gw_fl = 1.0f, gw_fr = 1.0f, gw_bl = 1.0f, gw_br = 1.0f;
                float gw_lf = 1.0f, gw_la = 1.0f, gw_rf = 1.0f, gw_ra = 1.0f;
                
                auto check_pair_g = [](int diff1, int diff2, float& w1, float& w2) {
                    if (diff1 < 0 || diff2 < 0 || diff1 >= 800 || diff2 >= 800) return;
                    int sensor_diff = std::abs(diff1 - diff2);
                    if (sensor_diff > 150) {
                        if (diff1 > diff2) { w1 = 0.0f; w2 = 1.5f; }
                        else { w2 = 0.0f; w1 = 1.5f; }
                    }
                };
                
                check_pair_g(gd_fl, gd_fr, gw_fl, gw_fr);
                check_pair_g(gd_bl, gd_br, gw_bl, gw_br);
                check_pair_g(gd_lf, gd_la, gw_lf, gw_la);
                check_pair_g(gd_rf, gd_ra, gw_rf, gw_ra);
                
                auto calc_deadband_g = [&]() -> int {
                    return is_precision_layer ? 10 : 30;
                };
                
                auto get_grad_err = [&](int diff, uint16_t actual_reading, bool use_sensor, float sensor_weight) -> uint32_t {
                    if (diff < 0) return 0;
                    if (sensor_weight <= 0.0f) return 0;
                    if (diff >= 800) {
                        gradientValidCount++;
                        return static_cast<uint32_t>((use_sensor ? 800.0f : 160.0f) * sensor_weight);
                    }
                    int base_db = calc_deadband_g();
                    int deadband = base_db;
                    if (actual_reading > 600 && actual_reading < 2000) {
                        int rate = is_precision_layer ? 30 : 15;
                        deadband = base_db + (actual_reading - 600) / rate;
                    }
                    int adj_diff = std::max(0, diff - deadband);
                    if (!use_sensor && adj_diff > 500) return 0;
                    gradientValidCount++;
                    return static_cast<uint32_t>(adj_diff * (use_sensor ? 1.0f : 0.2f) * sensor_weight);
                };
                
                gradientError += get_grad_err(gd_fl, readings.front_left, g_front, gw_fl);
                gradientError += get_grad_err(gd_fr, readings.front_right, g_front, gw_fr);
                gradientError += get_grad_err(gd_bl, readings.back_left, g_back, gw_bl);
                gradientError += get_grad_err(gd_br, readings.back_right, g_back, gw_br);
                gradientError += get_grad_err(gd_lf, readings.left_forward, g_left, gw_lf);
                gradientError += get_grad_err(gd_la, readings.left_aft, g_left, gw_la);
                gradientError += get_grad_err(gd_rf, readings.right_forward, g_right, gw_rf);
                gradientError += get_grad_err(gd_ra, readings.right_aft, g_right, gw_ra);
                
                if (gradientValidCount >= 2 && gradientError < minError) {
                    minError = gradientError;
                    bestCandidate = gradient;
                    
                    if (gradientError < top3[2].error) {
                        top3[2] = {gradient, gradientError};
                        if (top3[2].error < top3[1].error) {
                            std::swap(top3[1], top3[2]);
                            if (top3[1].error < top3[0].error) {
                                std::swap(top3[0], top3[1]);
                            }
                        }
                    }
                }
            }
        }
        
        // WEIGHTED AVERAGE OF TOP-3 PARTICLES (for precision layers)
        if (is_precision_layer && top3[0].error < 0xFFFFFFFF && top3[1].error < 0xFFFFFFFF) {
            double w0 = 1.0 / std::max(1UL, static_cast<unsigned long>(top3[0].error));
            double w1 = 1.0 / std::max(1UL, static_cast<unsigned long>(top3[1].error));
            double w2 = (top3[2].error < 0xFFFFFFFF) ? 1.0 / std::max(1UL, static_cast<unsigned long>(top3[2].error)) : 0.0;
            double total_w = w0 + w1 + w2;
            
            double avg_x = (w0 * top3[0].p.x + w1 * top3[1].p.x + w2 * top3[2].p.x) / total_w;
            double avg_y = (w0 * top3[0].p.y + w1 * top3[1].p.y + w2 * top3[2].p.y) / total_w;
            
            bestCandidate.x = static_cast<int16_t>(std::round(avg_x));
            bestCandidate.y = static_cast<int16_t>(std::round(avg_y));
            
            if (is_particle_valid(bestCandidate)) {
                raycast(bestCandidate);
            }
        }
        
        bestInLayer = bestCandidate;
    }
    
    _bestParticle = bestInLayer;
}
