
#ifndef OdometryTracker_h
#define OdometryTracker_h

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include "ID.h"

class OdometryTracker{

private:

    bool _start_left, _start_right;

    int _left_count, _right_count;

public:

    double x, y, z, th;

    double vx, vy, vz, vth;

    unsigned long last_update_time;

    unsigned long last_report_time;

    double v_left, v_right;

    bool changed = false;

    OdometryTracker(){
        reset();
    }

    void reset(){
        _start_left = true;
        _start_right = true;
        _left_count = 0;
        _right_count = 0;
        x = 0;
        y = 0;
        z = 0;
        th = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        vth = 0; //rad/sec
        last_update_time = 0;
        last_report_time = 0;
        v_left = 0;
        v_right = 0;
    }

    void update_left(int count){
        if (!_start_left) {
            v_left = (count - _left_count) * METERS_PER_COUNT;
            update();
        }
        _start_left = false;
        _left_count = count;
    }

    void update_right(int count){
        if (!_start_right) {
            v_right = (count - _right_count) * METERS_PER_COUNT;
            update();
        }
        _start_right = false;
        _right_count = count;
    }

    void update(){
        if (last_update_time > 0) {
            changed = true;

            // compute odometry in a typical way given the velocities of the robot
            // http://answers.ros.org/question/231942/computing-odometry-from-two-velocities/?answer=231954#post-id-231954
            double dt = (millis() - last_update_time)/1000.;
            
            // Calculate velocities.
            vx = ( v_right + v_left ) / 2;
            vy = 0; // we assume no left/right sliding
            vth = ( v_right - v_left ) / TORSO_TREAD_WIDTH_METERS;
            
            // Use velocities to update absolute position and rotation.
            double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            double delta_th = vth * dt;
            x += delta_x;
            y += delta_y;
            th += delta_th;

        }
        last_update_time = millis();
    }
};

#endif
