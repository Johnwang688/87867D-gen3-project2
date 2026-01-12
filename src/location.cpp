#include "bot.hpp"
#include "drivetrain.hpp"
#include "location.hpp"
using namespace vex;


struct SensorConfig {
    float offX, offY, relativeAngle;
};

static const SensorConfig sensor_layout[8] = {
    {static_cast<float>(FRONT_LEFT_OFFSET_X),  static_cast<float>(FRONT_LEFT_OFFSET_Y),   0.0f},  // 0: front_left
    {static_cast<float>(FRONT_RIGHT_OFFSET_X), static_cast<float>(FRONT_RIGHT_OFFSET_Y),  0.0f},  // 1: front_right
    {static_cast<float>(BACK_LEFT_OFFSET_X),   static_cast<float>(BACK_LEFT_OFFSET_Y),   180.0f}, // 2: back_left
    {static_cast<float>(BACK_RIGHT_OFFSET_X),  static_cast<float>(BACK_RIGHT_OFFSET_Y),  180.0f}, // 3: back_right
    {static_cast<float>(LEFT_FORWARD_OFFSET_X), static_cast<float>(LEFT_FORWARD_OFFSET_Y), 90.0f}, // 4: left_forward
    {static_cast<float>(LEFT_AFT_OFFSET_X),     static_cast<float>(LEFT_AFT_OFFSET_Y),     90.0f}, // 5: left_aft
    {static_cast<float>(RIGHT_FORWARD_OFFSET_X), static_cast<float>(RIGHT_FORWARD_OFFSET_Y), -90.0f},// 6: right_forward
    {static_cast<float>(RIGHT_AFT_OFFSET_X),    static_cast<float>(RIGHT_AFT_OFFSET_Y),    -90.0f} // 7: right_aft
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
    _updateTask(), _taskRunning(false),
    _bestParticle{0, 0, 0, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, true}
{
    _particles.reserve(9);
}

void Location::reset(std::int16_t x, std::int16_t y, std::int16_t heading) {
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

    // 2. Get the current heading from the IMU
    // We treat the IMU as the "Source of Truth" for rotation
    double current_imu_heading = _imu.get_heading();
    
    // 3. Update 'last' values for the next cycle
    _last_left_encoder = left_dist;
    _last_right_encoder = right_dist;

    // 4. Calculate the local movement vector
    // Using the average heading during the move (Arc Length approximation)
    // Handle heading wrap-around for correct averaging (e.g., 358° and 2° should average to 0°, not 180°)
    double avg_heading_deg = helpers::average_heading(_heading, current_imu_heading);
    double avg_heading_rad = math::to_rad(avg_heading_deg);
    
    Particle delta;
    delta.x = static_cast<int16_t>(_x + (d_center * cos(avg_heading_rad)));
    delta.y = static_cast<int16_t>(_y + (d_center * sin(avg_heading_rad)));
    delta.heading = static_cast<int16_t>(current_imu_heading);

    return delta; // This is our 'Guess' for the search function
}

float Location::calculate_intersection(float s_x, float s_y, float angle_deg, const Line& wall) {
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
}

bool Location::is_particle_valid(const Particle& particle) {
    // Check if particle center is within bounds
    if (particle.x < -OUT_OF_BOUNDS || particle.x > OUT_OF_BOUNDS || 
        particle.y < -OUT_OF_BOUNDS || particle.y > OUT_OF_BOUNDS) {
        return false;
    }
    
    // Check if all sensor positions are within field bounds
    float rad_h = math::to_rad(particle.heading);
    float cos_h = std::cos(rad_h);
    float sin_h = std::sin(rad_h);
    
    for (int i = 0; i < 8; ++i) {
        float s_x = particle.x + (sensor_layout[i].offX * cos_h - sensor_layout[i].offY * sin_h);
        float s_y = particle.y + (sensor_layout[i].offX * sin_h + sensor_layout[i].offY * cos_h);
        
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

    float rad_h = math::to_rad(particle.heading);
    float cos_h = std::cos(rad_h);
    float sin_h = std::sin(rad_h);

    for (int i = 0; i < 8; ++i) {
        float s_x = particle.x + (sensor_layout[i].offX * cos_h - sensor_layout[i].offY * sin_h);
        float s_y = particle.y + (sensor_layout[i].offX * sin_h + sensor_layout[i].offY * cos_h);

        float world_angle = particle.heading + sensor_layout[i].relativeAngle;

        float min_dist = 2000.0f;
        for (const auto& wall : map) {
            float d = calculate_intersection(s_x, s_y, world_angle, wall);
            if (d < min_dist) min_dist = d;
        }

        *p_readings[i] = static_cast<uint16_t>(min_dist);
    }
}

void Location::update() {
    Particle guess = estimate();
    Reading readings = capture_readings();
    search(guess, readings);
    
    // Lock mutex when updating shared position data
    _positionMutex.lock();
    _x = _bestParticle.x;
    _y = _bestParticle.y;
    _heading = _bestParticle.heading;
    _positionMutex.unlock();
}

void Location::search(Particle guess, Reading readings) {
    Particle bestInLayer = guess;
    bestInLayer.valid = true;

    for (int layer = 0; layer < LAYERS; ++layer) {  
        int16_t step = INITIAL_STEP_SIZE >> layer; 
        _particles.clear();

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                Particle p = bestInLayer;
                p.x += dx * step;
                p.y += dy * step;
                
                // Validate particle before raycast
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
        for (const auto& p : _particles) {
            if (!p.valid) continue;
            
            uint32_t currentError = 0;

            auto get_err = [](uint16_t sim, uint16_t act) -> uint16_t {
                if (act >= 2000 && sim >= 2000) return 0;
                return std::abs(static_cast<int>(sim) - static_cast<int>(act));
            };

            currentError += get_err(p.front_left,    readings.front_left);
            currentError += get_err(p.front_right,   readings.front_right);
            currentError += get_err(p.back_left,     readings.back_left);
            currentError += get_err(p.back_right,    readings.back_right);
            currentError += get_err(p.left_forward,  readings.left_forward);
            currentError += get_err(p.left_aft,      readings.left_aft);
            currentError += get_err(p.right_forward, readings.right_forward);
            currentError += get_err(p.right_aft,     readings.right_aft);

            if (currentError < minError) {
                minError = currentError;
                bestInLayer = p;
            }
        }
    }
    _bestParticle = bestInLayer;
}
