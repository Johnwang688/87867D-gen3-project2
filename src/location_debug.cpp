#include "location_debug.hpp"

namespace debug {
    void print_location(){
        using namespace bot::mcl;
        printf("------------------------------------------\n");
        printf("Location: x: %d, y: %d, heading: %d\n", location.get_x(), location.get_y(), location.get_heading());
        printf("==========================================\n");
    }

    void print_location_controller(){

    }

    void print_sensor_data(){
        using namespace bot::sensors;
        double left_distance_forward_reading = left_distance_forward.objectDistance(vex::mm);
        double left_distance_aft_reading = left_distance_aft.objectDistance(vex::mm);
        double right_distance_forward_reading = right_distance_forward.objectDistance(vex::mm);
        double right_distance_aft_reading = right_distance_aft.objectDistance(vex::mm);
        double front_distance_left_reading = front_distance_left.objectDistance(vex::mm);
        double front_distance_right_reading = front_distance_right.objectDistance(vex::mm);
        double back_distance_left_reading = back_distance_left.objectDistance(vex::mm);
        double back_distance_right_reading = back_distance_right.objectDistance(vex::mm);


        printf("----------------------------------------\n");

        printf("Left distance forward: %f\n", left_distance_forward_reading);
        printf("Left distance aft: %f\n", left_distance_aft_reading);
        printf("Right distance forward: %f\n", right_distance_forward_reading);
        printf("Right distance aft: %f\n", right_distance_aft_reading);
        printf("Front distance left: %f\n", front_distance_left_reading);
        printf("Front distance right: %f\n", front_distance_right_reading);
        printf("Back distance left: %f\n", back_distance_left_reading);
        printf("Back distance right: %f\n", back_distance_right_reading);

        printf("========================================\n");
    }
}