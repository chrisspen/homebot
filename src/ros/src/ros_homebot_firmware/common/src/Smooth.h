/*
 *
 * Functions for creating smooth servo control signals.
 *
 * */
#ifndef Smooth_h
#define Smooth_h

#include <math.h>

double scale(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / float(in_max - in_min) + out_min;
}

double smootherstep(double x, double edge0=0., double edge1=1.){
    // Scale, bias and saturate x to 0..1 range
    x = min(max((x - edge0)/(edge1 - edge0), 0.0), 1.0);
    // Evaluate polynomial
    //return x*x*(3 - 2*x);
    return x*x*x*(x*(x*6 - 15) + 10);
}

// Derived from test_smoothing.py
int get_servo_signal(int start, int end, int speed, double t){
	/* Calculates a servo signal to simulate a smooth servo movement.
	 * Given a start and end servo position in degrees, speed of change, and time index,
	 * calculates the intermediate position.
	 * Designed to be called in a loop repeatedly during servo movement until the end position
	 * is achieved.
	 *
	 * start = start position, should be a value in 0:180
	 * end = end position, should be a value in 0:180
	 * speed = rate of position change in sec/degree
	 * t = relative time since the movement started
	 *
	 * */
    double dist = abs(end - start);
    double steps = int(round(dist/speed)) + 1;
    //Test::out->println(String("t/steps=")+String(t/steps));
    double v = smootherstep(t/steps);
    //Test::out->println(String("v0=")+String(v));
    //v = (int)round(map(v, 0, 1, start, end));
    v = int(round(scale(v, 0, 1, start, end)));
    return v;
}

#endif
