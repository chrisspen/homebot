#! /usr/bin/env python
import time
import random
import sys
import traceback
from threading import RLock

import roslib
#roslib.load_manifest('ros_homebot')
import rospy
import actionlib

import ros_homebot.msg
from ros_homebot_python import constants as c
from ros_homebot_python.node import (
    subscribe_to_topic,
    get_service_proxy,
    get_topic_name,
    packet_to_service_type,
    packet_to_service_request_type,
)
from ros_homebot_python import utils

class Buffer(object):
    
    def __init__(self, length=2):
        self.values = []
        self.length = length
        
    def add(self, value):
        self.values.append(value)
        if len(self.values) > self.length:
            self.values.pop(0)
        
    def get(self):
        return sum(self.values)/float(len(self.values))
        
    def __unicode__(self):
        return unicode(self.get())
        
    def __repr__(self):
        return unicode(self)

class Infrequent(object):
    
    def __init__(self, period=1):
        self.period = period
        self.last = time.time()
        
    def ready(self):
        ret = (time.time() - self.last) > self.period
        if ret:
            self.last = time.time()
        return ret

class MotionServer:
  
    def __init__(self, name):
        
        self.running = True
        
        self.motor_speed = c.MOTOR_EIGTH_SPEED
        
        self._lock = RLock()
        
        self._last_action = None, time.time() # action, last set
        
        self.edge_states = [0]*3
        self.bumper_states = [0]*3
        self.ultrasonic_states = [Buffer(length=1) for _ in range(3)]
        
        self.last_edge_update = 0
        self.last_bumper_update = 0
        self.last_ultrasonic_update = 0
        
        self.ultrasonic_update_start = time.time()
        self.ultrasonic_update_count = 0
        
        self.enable_ultrasonics()
        
        subscribe_to_topic(c.TORSO, c.ID_EDGE, self.on_edge_update)
        subscribe_to_topic(c.TORSO, c.ID_BUMPER, self.on_bumper_update)
        subscribe_to_topic(c.TORSO, c.ID_ULTRASONIC, self.on_ultrasonic_update)
        subscribe_to_topic(c.TORSO, c.ID_STATUS_BUTTON, self.on_status_button_update)
        #TODO:accel/gyro?
        
        rospy.on_shutdown(self.shutdown)
        
        print 'Starting servers...'
        
        self.wander_server = actionlib.SimpleActionServer(
            c.MOTION_WANDER,
            ros_homebot.msg.WanderAction,
            self.execute_wander,
            False)
        self.wander_server.start()
        
        self.forward_server = actionlib.SimpleActionServer(
            c.MOTION_FORWARD_X_MM,
            ros_homebot.msg.ForwardAction,
            self.execute_forward,
            False)
        self.forward_server.start()
        
        self.turn_server = actionlib.SimpleActionServer(
            c.MOTION_TURN_X_DEGREES,
            ros_homebot.msg.TurnAction,
            self.execute_turn,
            False)
        self.turn_server.start()
        
        print 'Ready!'
    
    def cancel_all(self, exclude=None):
        with self._lock:
            for motion_slug, motion_name in c.MOTIONS:
                if exclude != motion_slug:
                    server = getattr(self, motion_name + '_server', None)
                    if (server and server.is_active() and server.current_goal.get_goal()
                    and server.current_goal != server.next_goal):
                        server.current_goal.set_canceled(
                            None,
                            "This goal was canceled because another goal was received "
                            "by the simple action server")
#                     action_cls = getattr(ros_homebot.msg, '%sAction' % motion_name.title())
#                     client = actionlib.SimpleActionClient(motion_slug, action_cls)
#                     client.wait_for_server()
#                     client.cancel_all_goals()
    
    def enable_ultrasonics(self):
        print 'Enabling ultrasonics...'
        get_service_proxy(c.TORSO, c.ID_SONAR_POWER)(1)
    
    def disable_ultrasonics(self):
        get_service_proxy(c.TORSO, c.ID_SONAR_POWER)(0)
        
    def on_edge_update(self, msg):
        print 'edge:', msg.index, msg.state
        self.last_edge_update = time.time()
        self.edge_states[msg.index-1] = msg.state
        
    def on_bumper_update(self, msg):
        print 'bumper:', msg.index, msg.state
        self.last_bumper_update = time.time()
        self.bumper_states[msg.index-1] = msg.state
        
    def on_ultrasonic_update(self, msg):
#         print 'ultrasonic:', msg.index, msg.distance
        self.last_ultrasonic_update = time.time()
        #print 'msg.index:', msg.index, msg.distance
        self.ultrasonic_states[msg.index-1].add(msg.distance)
        if msg.index == 1:
            self.ultrasonic_update_count += 1
    
    @property
    def ultrasonic_update_rate(self):
        # Returns updates/sec.
        td = time.time() - self.ultrasonic_update_start
        cnt = self.ultrasonic_update_count
        return cnt/td
        
#     @property
#     def ultrasonic_update_period(self):
#         return 60./self.ultrasonic_update_rate
    
    def on_status_button_update(self, msg):
        if msg.state:
            self.running = False
    
    def check_action(self, action, rate=1):
        """
        Prevents an action from being executed more than once per rate seconds.
        """
        last_action, last_set = self._last_action
        if (last_action != action
        or (last_action == action and abs(time.time() - last_set) >= rate)):
            self._last_action = action, time.time()
            return True
        return False
    
    def stop(self):
        if not self.check_action(c.MOTOR_BREAK):
            return
        print 'stop'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        print 'proxy:', proxy
        proxy(left=0, right=0)
    
    def reverse(self):
        if not self.check_action(c.MOTOR_REVERSE):
            return
        print 'reverse'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=-self.motor_speed, right=-self.motor_speed)
    
    def go_forward(self):
        if not self.check_action(c.MOTOR_FORWARD):
            return
        print 'forward'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=self.motor_speed, right=self.motor_speed)
    
    def turn_left(self):
        if not self.check_action(c.MOTOR_TURN_CCW):
            return
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=-self.motor_speed, right=+self.motor_speed)
     
    def turn_right(self):
        if not self.check_action(c.MOTOR_TURN_CW):
            return
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=+self.motor_speed, right=-self.motor_speed)
        
    def pivot_left_cw(self):
        if not self.check_action(c.MOTOR_PIVOT_LEFT_CW):
            return
        print 'pivot_left_cw'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=0, right=-self.motor_speed)
        
    def pivot_left_ccw(self):
        if not self.check_action(c.MOTOR_PIVOT_LEFT_CCW):
            return
        print 'pivot_left_ccw'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=0, right=+self.motor_speed)
        
    def pivot_right_cw(self):
        if not self.check_action(c.MOTOR_PIVOT_RIGHT_CW):
            return
        print 'pivot_right_cw'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=+self.motor_speed, right=0)
        
    def pivot_right_ccw(self):
        if not self.check_action(c.MOTOR_PIVOT_RIGHT_CCW):
            return
        print 'pivot_right_ccw'
        proxy = get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)
        proxy(left=-self.motor_speed, right=0)
    
    def execute_forward(self, goal):
        """
        Drives the platform straight forward a fixed distance.
        
        Measured Accuracy of +/- 1cm. 
        """
    
        self.cancel_all(exclude=c.MOTION_FORWARD_X_MM)
        
        travel_time = utils.linear_travel_time(
            speed=utils.relative_to_absolute_speed(self.motor_speed),
            distance=goal.distance_mm * c.MM,
            start_speed=0*c.MM/c.SEC,
            # unnecessary, because stop() will automatically handle deceleration
            #end_speed=0*c.MM/c.SEC,
        )
#         print 'linear travel_time:', travel_time
        
        if goal.distance_mm > 0:
            self.go_forward()
        else:
            self.reverse()
            
        t0 = time.time()
        while (self.running and not self.wander_server.is_preempt_requested()
        and (time.time() - t0)*c.SEC < travel_time):
            time.sleep(0.1)
        self.stop()
        
        result = ros_homebot.msg.ForwardResult()
        self.forward_server.set_succeeded(result)
    
    def execute_turn(self, goal):
        """
        Rotates the platform about the center axis by a fixed degree.
        
        Measured Accuracy of +/- 10 degrees. 
        """
    
        self.cancel_all(exclude=c.MOTION_TURN_X_DEGREES)
        
        travel_time = utils.rotational_travel_time(
            speed=utils.relative_to_absolute_speed(self.motor_speed),
            degrees=goal.degrees * c.DEG,
        )
        travel_time *= 2
#         print 'degrees:', goal.degrees
#         print 'turn travel_time:', travel_time
        
        if goal.degrees > 0:
            self.turn_right()
        else:
            self.turn_left()
            
        t0 = time.time()
        while (self.running and not self.wander_server.is_preempt_requested()
        and (time.time() - t0)*c.SEC < travel_time):
            time.sleep(0.1)
        self.stop()
        
        result = ros_homebot.msg.TurnResult()
        self.turn_server.set_succeeded(result)
    
    def execute_pivot(self, goal):
        """
        Rotates the platform about a tread axis by a fixed degree.
        """
    
        self.cancel_all(exclude=c.MOTION_PVIOT_X_DEGREES)
        
        travel_time = utils.pivot_travel_time(
            speed=utils.relative_to_absolute_speed(self.motor_speed),
            distance=goal.distance_mm * c.MM,
        )
        print 'pivot travel_time:', travel_time
        
        self.go_forward()
        t0 = time.time()
        while (self.running and not self.wander_server.is_preempt_requested()
        and (time.time() - t0)*c.SEC < travel_time):
            time.sleep(0.1)
        self.stop()
        
        result = ros_homebot.msg.PivotResult()
        self.forward_server.set_succeeded(result)
        
    def execute_wander(self, goal):
        """
        Randomly drives the platform around, using sensors to avoid running into obstacles.
        """
        
        self.cancel_all(exclude=c.MOTION_WANDER)
        
        try:
            print 'Wandering...', time.time()
            delayer = Infrequent(period=1)
            t0 = time.time()
            checks = 0
            while self.running and not self.wander_server.is_preempt_requested():
                checks += 1
                
                check_rate = checks/(time.time() - t0)
                
                # If we go too long without sensor updates, then stop and wait.
                if time.time() - self.last_ultrasonic_update > 5:
                    self.enable_ultrasonics()
                    print 'Waiting because no data.'
                    self.stop()
                    time.sleep(1)
                    continue
                
                # If we hit something with our bumper, stop.
#                 if any(self.bumper_states):
#                     if self.bumper_states[0]:
#                         print 'Bumper hit left.'
#                         self.reverse()
#                         time.sleep(1)
#                         self.pivot_left_cw()
#                         time.sleep(1)
#                         self.go_forward()
#                     elif self.bumper_states[1]:
#                         print 'Bumper hit center.'
#                         self.reverse()
#                         time.sleep(1)
#                         if random.random() > 0.5:
#                             self.pivot_left_cw()
#                         else:
#                             self.pivot_right_ccw()
#                         time.sleep(random.randint(1, 5))
#                         self.go_forward()
#                     elif self.bumper_states[2]:
#                         print 'Bumper hit right.'
#                         self.reverse()
#                         time.sleep(1)
#                         self.pivot_right_ccw()
#                         time.sleep(1)
#                         self.go_forward()
#                     continue
                
                # If we detect proximity via ultrasound, change course.
                ultrasonic_states_filtered = \
                    [1 if _.get() < 10 else 0 for _ in self.ultrasonic_states]
                if delayer.ready():
                    print 'check_rate:', check_rate
                    print 'Ultrasonic update rate:', self.ultrasonic_update_rate
                    print 'ultrasonic_states_raw:', self.ultrasonic_states
                    print 'ultrasonic_states_filtered:', ultrasonic_states_filtered
                if any(ultrasonic_states_filtered):
                    print 'STOP!'
                    self.stop()
                    time.sleep(1)
#                     if ultrasonic_states_filtered[0]:
#                         print 'Ultrasonic hit left.'
#                         self.reverse()
#                         time.sleep(1)
#                         self.pivot_left_cw()
#                         time.sleep(random.randint(1, 2))
#                         self.go_forward()
#                     elif ultrasonic_states_filtered[1]:
#                         print 'Ultrasonic hit center.'
#                         self.reverse()
#                         time.sleep(1)
#                         if random.random() > 0.5:
#                             self.pivot_left_cw()
#                         else:
#                             self.pivot_right_ccw()
#                         time.sleep(random.randint(1, 5))
#                         self.go_forward()
#                     elif ultrasonic_states_filtered[2]:
#                         print 'Ultrasonic hit right.'
#                         # right-hit
#                         self.reverse()
#                         time.sleep(1)
#                         self.pivot_right_ccw()
#                         time.sleep(random.randint(1, 2))
#                         self.go_forward()
                    continue
                
                #TODO:edge? 1=cliff, 0=ground
                
                # Otherwise, go forward.
                self.go_forward()
                    
                #time.sleep(0.1)
                
            #self._as.publish_feedback(self._feedback)
            result = ros_homebot.msg.WanderResult()
            self.wander_server.set_succeeded(result)
            
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            
        self.shutdown()
        print 'Action done.'

    def shutdown(self):
        self.disable_ultrasonics()
        self.stop()

if __name__ == '__main__':
    rospy.init_node('wander')
    MotionServer(rospy.get_name())
    rospy.spin()
