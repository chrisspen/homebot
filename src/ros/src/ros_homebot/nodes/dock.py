#!/usr/bin/env python
"""
2018-4-24 This manages navigating the robot to its docking station when it's a few feet away and in line of sight to the docking station.

Usage:

    export ROS_MASTER_URI=http://rae.local:11311;
    rosrun ros_homebot dock.py

This will begin the docking process, which is fully automated. The node will run until it arrives at one of the following terminal states:

1. it successfully docks with a powered docking station
2. it successfully docks with an unpowered docking station
3. it is unable to dock after several failed attempts
4. a safety measure is triggered, such as detection of a cliff or mechanical fault, or failure to detect the docking code

You can cancel the docking procedure by sending the topic:

"""
from __future__ import print_function

from functools import partial
from commands import getoutput
import time
import re

import rospy
import roslaunch
from std_srvs.srv import Empty, EmptyResponse

from ros_qr_tracker.srv import SetTarget
from ros_qr_tracker.msg import Percept

import ros_homebot_msgs.srv

from common import subscribe, publish # pylint: disable=relative-import

class State(object):
    pass

class DockNode(object):

    # We're looking for the QR code identifying our docking station.
    SEARCHING = 'searching'

    # We're moving ourselves so we align with the center of the docking station.
    CENTERING = 'centering'

    # We've centered and are now moving forward towards the QR code and dock coupling until the EP1 and EP2 sensors register a connection.
    APPROACHING = 'approaching'

    # We've successfully connected with the docking station.
    CONNECTED = 'centering'

    # We've failed to connect, probably because our approach was misaligned, so backup so we can retry our approach.
    RETREATING = 'retreating'

    def __init__(self, **kwargs):
        rospy.init_node('dock', log_level=rospy.DEBUG)

        self.state = State()
        self.state.index = self.SEARCHING
        self.state.ep0 = None # External power voltage present.
        self.state.ep1 = None # External power magnet present.
        self.state.pan_degrees = None
        self.state.qr_matches = None
        self.state.motor_target_a = None
        self.state.motor_target_b = None

        self.cancelled = False

        self.audible = kwargs.pop('audible')

        self.qr_tracker_process = None

        self.qr_code = "d=homebot,sd=dock,w=55,h=55"

        rospy.on_shutdown(self.on_shutdown)

        # Launch qr tracker
        self.start_qr_tracker()
        self.qr_tracker_set_target_srv = rospy.ServiceProxy('/qr_tracker/set_target', SetTarget)
        self.qr_tracker_set_target_srv(self.qr_code)

        # Track external power voltage present.
        subscribe('/torso_arduino/power/external/0', partial(self._set_state, name='ep0'))

        # Track external power magnet present.
        subscribe('/torso_arduino/power/external/1', partial(self._set_state, name='ep1'))

        # Track edge sensors, to track emergency abort when user lifts us up.
        subscribe('/torso_arduino/edge/0', partial(self._set_state, name='edge0'))
        subscribe('/torso_arduino/edge/1', partial(self._set_state, name='edge1'))
        subscribe('/torso_arduino/edge/2', partial(self._set_state, name='edge2'))

        # Track main drive motor activity.
        subscribe('/torso_arduino/motor/target/a', partial(self._set_state, name='motor_target_a'))
        subscribe('/torso_arduino/motor/target/b', partial(self._set_state, name='motor_target_b'))

        # Track the position of the head's pan degree.
        subscribe('/head_arduino/pan/degrees', partial(self._set_state, name='pan_degrees'))

        # Track QR matches.
        subscribe('/qr_tracker/matches', partial(self._set_state, name='qr_matches'))

        self.say = rospy.ServiceProxy('/sound/say_eventually', ros_homebot_msgs.srv.TTS)

        rospy.Service('~cancel', Empty, self.on_cancel)

        self.force_sensors()

        dock_iter = self.dock_iter()
        while not rospy.is_shutdown() and not self.cancelled:
            # time.sleep(1)
            try:
                dock_iter.next()
            except StopIteration:
                break
        time.sleep(3)

    def is_torso_moving(self):
        return self.state.motor_target_a or self.state.motor_target_b

    def is_torso_stopped(self):
        return not self.is_torso_moving()

    def halt_motors(self):
        """
        Tells our motors to immediately stop.
        """
        publish('/torso_arduino/halt')# std_msgs/Empty

    def on_cancel(self, msg=None):
        """
        Terminates any running docking process and results in this node exiting.
        """
        self.cancelled = True
        self.on_shutdown()
        return EmptyResponse()

    def on_shutdown(self):
        """
        Called when the ROS master signals a shutdown, or our parent signals a cancellation of the docking process.
        """
        self.halt_motors()
        rospy.loginfo('Shutting down...')
        if self.qr_tracker_process:
            self.qr_tracker_process.stop()
        rospy.loginfo('Done.')

    def start_qr_tracker(self):
        """
        Launches a child node to detect the QR code from the primary camera video feed.
        This code will tell us where the docking station is.
        """
        if getoutput('rosnode list | grep /qr_tracker'):
            rospy.logwarn('QR tracker already running.')
            return

        node = roslaunch.core.Node(
            package='ros_qr_tracker',
            node_type='qr_tracker.py',
            name='qr_tracker',
            args='_topic:=/raspicam/image/compressed _start:=1 _mode:=2',
            output='screen')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        rospy.loginfo('qr_tracker.py is_alive: %s' % process.is_alive())
        time.sleep(5)

    def _set_state(self, msg, name):
        """
        This is a helper routine to catch all the state change information, and update our local state cache accordingly.
        """
        data = msg.data
        if isinstance(msg, Percept):
            data = msg
        rospy.loginfo('setting state: %s=%s(type=%s)' % (name, data, type(msg)))
        assert hasattr(self.state, name), 'State cache has no attribute "%s".' % name
        setattr(self.state, name, data)
        setattr(self.state, name+'_update', time.time())

    def force_sensors(self):
        """
        Tells the embedded controllers to give us a full update of all sensor states, so we can update our local cache.
        """
        # publish('/head_arduino/force_sensors')
        for _ in range(10):
            publish('/torso_arduino/force_sensors')
            if self.state.ep0 is not None and self.state.ep1 is not None:
                break
            time.sleep(1)

    def center_head(self):
        """
        Causes the head to pan to its center position.
        """
        publish('/head_arduino/tilt/set', 90)
        time.sleep(2)
        publish('/head_arduino/pan/set', 315)
        time.sleep(2)
        publish('/head_arduino/pan/set', 356)
        time.sleep(2)

    def qr_code_found(self):
        """
        Returns true if we see a QR code matching that of our docking station.
        Returns false otherwise.
        """
        return self.state.qr_matches is not None

    def is_docked(self):
        """
        Return true if we're docked to a powered docking station. Returns false otherwise.
        """
        return self.state.ep0 and self.state.ep1

    def is_dead_docked(self):
        """
        Return true if we detect the docking station magnet, but we're not detecting power.

        This may mean the docking station is not plugged in to a power supply, or the magnetic coupling has snagged
        in such a way that an electrical connection is not being made.

        In the later case, this can usually be fixed by using the motors to "wiggle" the robot slightly left and right.
        If the robot is too of-center for this to work, the only other option is to completely abort the docking, reverse,
        and retry the full docking procedure, hoping the next try will obtain better centering.

        Returns false otherwise.
        """
        return not self.state.ep0 and self.state.ep1

    def center_qr_in_view(self):
        """
        Rotates the head until the QR code is roughly in the center of view.
        """
        # Note, our motors don't have good precision, and get stuck at small intervals, so the most we can adjust by are in 10 degree increments.
        angle_of_adjustment = 10
        # Increase angle => rotate CW
        # Decreate angle => rotate CCW
        if not self.state.qr_matches:
            rospy.logerr('Cannot center QR code. None found.')
            return
        for _ in range(10):
            x = (self.state.qr_matches.a[0] + self.state.qr_matches.b[0] + self.state.qr_matches.c[0] + self.state.qr_matches.d[0])/4.
            # y = (self.state.qr_matches.a[1] + self.state.qr_matches.b[1] + self.state.qr_matches.c[1] + self.state.qr_matches.d[1])/4.
            width = self.state.qr_matches.width
            # height = self.state.qr_matches.height

            # Ideally, this should be 0.5, indicating the QR code is exactly in the center of the horizontal view.
            # In practice, we'll likely never accomplish this, but we'll try to get it within [0.4:0.6]
            position_ratio = x/float(width)
            rospy.loginfo('position_ratio: %s' % position_ratio)
            if 0.4 <= position_ratio <= 0.6:
                break
            elif position_ratio < 0.4:
                # QR code is too far left of screen, so move head counter-clockwise to center it.
                new_angle = (self.state.pan_degrees - angle_of_adjustment) % 360
                publish('/head_arduino/pan/set', new_angle)
            elif position_ratio > 0.6:
                # QR code is too far right of screen, so move head clockwise to center it.
                new_angle = (self.state.pan_degrees + angle_of_adjustment) % 360
                publish('/head_arduino/pan/set', new_angle)
            rospy.loginfo('new angle: %s' % new_angle)
            time.sleep(1)

    def announce_status(self, status):
        rospy.loginfo(status)
        if self.audible:
            self.say(re.sub('[^a-zA-Z0-9]+', ' ', status))

    def dock_iter(self):
        """
        This is the main docking procedure.
        This will stop iterating once the docking procedure has succeeded or reached a failure condition.
        """
        self.announce_status('Beginning dock procedure.')

        self.announce_status('Checking current state.')
        if self.is_docked():
            self.announce_status('We are already docked. Nothing to do.')
            return

        if self.is_dead_docked():
            self.announce_status('We sense the docking magnet, but no power, possibly from a previous failed docking attempt.')
            self.announce_status('Undocking to abort last attempt and retry.')
            self.undock()
            yield
            time.sleep(5)
            yield

        self.announce_status('Centering head.')
        self.center_head()
        yield

        self.announce_status('Searching for QR code.')
        angle = 356
        cnt = 0
        for _ in range(20):
            angle = (angle + 18) % 360
            self.announce_status('Scanning angle %s.' % angle)
            publish('/head_arduino/pan/set', angle)
            time.sleep(1)
            if self.qr_code_found():
                self.announce_status('QR code found.')
                break
        if not self.qr_code_found():
            self.announce_status('QR code could not be found. Docking terminated.')
            return

        self.announce_status('Centering QR code in view.')
        self.center_qr_in_view()
        yield
        self.announce_status('View centered.')

        self.announce_status('Freezing head angle.')
        publish('/head_arduino/pan/freeze/set', 1)
        yield

        self.announce_status('Angling body to be perpendicular to QR code.')
        # Calculate torso angle rotation needed so that the front faces inward by 90 degrees to the head position.
        # Note, a positive degree proceeds CW when looking down.
        # e.g. if we're facing -10 degrees to the left == 350 degrees, then we need to rotate the torso by -10 degrees to center,
        # and then another -90 degrees to be perpendicular.
        #TODO:estimate angle of offset of QR code
        if self.state.pan_degrees > 180:
            # Need to rotate CCW.
            head_angle_offset = self.state.pan_degrees - 360 -90
        else:
            # Need to rotate CW.
            head_angle_offset = self.state.pan_degrees + 90
        rospy.loginfo('head_angle_offset: %s' % head_angle_offset)
        publish('/torso_arduino/motor/rotate', head_angle_offset)
        yield

        # Wait until torso stops rotating.
        self.announce_status('Waiting until we have fully rotated.')
        time.sleep(3)
        while self.is_torso_moving():
            yield
            time.sleep(0.5)
            rospy.loginfo('Waiting...')
        self.announce_status('Body angled.')

        # rospy.loginfo('Moving body to be infront of QR code.')
        # rospy.loginfo('Body positioned in front.')

        # rospy.loginfo('Approaching dock.')

        # elif self.state.index == self.CENTERING:
            # #rotate body to parallel dock
            # #move sideways until perpendicular to dock
            # pass
        # elif self.state.index == self.APPROACHING:
            # #rotate body, keeping head pointed at dock, until body straight inline with dock
            # #move forward slowly in bursts until EP1 and EP2 register power connected
            # #disable forward motors in arduino when EP1 and EP2 set, timeout after 3 seconds
            # pass
        # elif self.state.index == self.RETREATING:
            # pass
        # time.sleep(1)
        self.announce_status('Dock procedure terminated.')

    def undock(self):
        publish('/torso_arduino/undock') # std_msgs/Empty

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--audible', default=False, action='store_true', help='If given audible status updates will be given.')
    args = parser.parse_args()
    DockNode(**args.__dict__)
