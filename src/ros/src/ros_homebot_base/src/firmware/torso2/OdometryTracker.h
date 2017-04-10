
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

    int x, y, z, th;

    int vx, vy, vz, vth;

    int deltaLeft, deltaRight;

    unsigned long last_update_time;

    unsigned long last_report_time;

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
        vth = 0;
        deltaLeft = 0;
        deltaRight = 0;
        last_update_time = 0;
        last_report_time = 0;
    }

    void update_left(int count){
        if (!_start_left) {
            deltaLeft = count - _left_count;
            vx = deltaLeft * METERS_PER_COUNT;
            update();
        }
        _start_left = false;
        _left_count = count;
    }

    void update_right(int count){
        if (!_start_right) {
            deltaRight = count - _right_count;
            vy = deltaRight * METERS_PER_COUNT;
            update();
        }
        _start_right = false;
        _right_count = count;
    }

    void update(){
        if (last_update_time > 0) {
            changed = true;

            // compute odometry in a typical way given the velocities of the robot
            float dt = (millis() - last_update_time)/1000.;
            float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            float delta_th = vth * dt;
            x += delta_x;
            y += delta_y;
            th += delta_th;

        }
        last_update_time = millis();
    }
};

#endif
