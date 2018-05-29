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
from math import pi
from threading import Thread
from thread import interrupt_main
import traceback

import rospy
from rospy.service import ServiceException
import roslaunch
from std_srvs.srv import Empty, EmptyResponse

from ros_qr_tracker.srv import SetTarget
from ros_qr_tracker.msg import Percept

import ros_homebot_msgs.srv

# pylint: disable=relative-import
from common import subscribe, publish
from state import State, TriggerTimeout


class Terminate(Exception):
    pass


class TorsoMotorError(Exception):
    pass


def normalize_angle_change(d1, d0):
    """
    Calculates the difference in degrees, taking into account 360 degree rollover.
    """
    change = d1 - d0
    if change > 180:
        change -= 360
    elif change < -180:
        change += 360
    return change


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
        node_name = kwargs.pop('__name', None) or 'dock_node'
        rospy.init_node(node_name, log_level=rospy.DEBUG)

        self.state = State()
        self.state.register('index', self.SEARCHING)

        self.cancelled = False

        self.audible = kwargs.pop('audible')

        self.say = rospy.ServiceProxy('/sound/say_eventually', ros_homebot_msgs.srv.TTS)

        self.announce_status('Initializing.')

        self.qr_tracker_process = None

        self.qr_code = "d=homebot,sd=dock,w=55,h=55"

        rospy.on_shutdown(self.on_shutdown)

        # Launch qr tracker
        self.start_qr_tracker()
        self.qr_tracker_set_target_srv = rospy.ServiceProxy('/qr_tracker/set_target', SetTarget)
        for _ in range(10):
            try:
                self.qr_tracker_set_target_srv(self.qr_code)
                break
            except ServiceException as exc:
                rospy.logerr('Unable to connect to qr_tracker: %s', exc)
                time.sleep(1)

        # Track external power voltage present.
        # subscribe('/torso_arduino/power_external_0', partial(self._set_state, name='ep0'))
        self.register('/torso_arduino/power_external_0')

        # Track external power magnet present.
        self.register('/torso_arduino/power_external_1')#, partial(self._set_state, name='ep1'))

        # Track edge sensors, to track emergency abort when user lifts us up.
        self.register('/torso_arduino/edge_0')
        self.register('/torso_arduino/edge_1')
        self.register('/torso_arduino/edge_2')

        self.register('/torso_arduino/imu_euler_z')

        # Track main drive motor activity.
        self.register('/torso_arduino/motor_target_a')
        self.register('/torso_arduino/motor_target_b')

        # Track the position of the head's pan degree.
        self.register('/head_arduino/pan_degrees')
        self.register('/head_arduino/pan_freeze_get')

        # Track QR matches.
        self.register('/qr_tracker/matches', time_to_stale=1.0)

        rospy.Service('~cancel', Empty, self.on_cancel)

        self.force_sensors()

        # Launch thread to check for emergency conditions and interrupt the main thread when appropriate.
        self.emergency_cancel_thread = Thread(target=self.emergency_cancel)
        self.emergency_cancel_thread.daemon = True
        self.emergency_cancel_thread.start()

        # while 1:
            # print('rotating...')
            # publish('/torso_arduino/motor_rotate', 90)
            # time.sleep(5)

        dock_iter = self.dock_iter()
        while not rospy.is_shutdown() and not self.cancelled:
            # time.sleep(1)
            try:
                dock_iter.next()
            except StopIteration:
                break
        time.sleep(3)

    def emergency_cancel(self, *args, **kwargs):
        """
        Monitors the state and interrupts the main thread if emergency conditions arise.
        """
        while 1:
            # Immediately halt docking procedure if we detect an edge.
            if self.state.torso_arduino_edge_0 or self.state.torso_arduino_edge_1 or self.state.torso_arduino_edge_2:
                rospy.logerr('Edge detected!')
                interrupt_main()
                return
            time.sleep(0.1)

    def register(self, topic, **kwargs):
        self.state.register(topic, **kwargs)
        subscribe(topic, partial(self._set_state, topic=topic))

    def _set_state(self, msg, topic):
        """
        This is a helper routine to catch all the state change information, and update our local state cache accordingly.
        """
        data = msg.data
        if isinstance(msg, Percept):
            # Percept messages from the QR code tracker are stored in their entirety.
            data = msg
        self.state.set(topic, data)

    def force_sensors(self):
        """
        Tells the embedded controllers to give us a full update of all sensor states, so we can update our local cache.
        """
        for _ in range(10):
            rospy.loginfo('Forcing sensors...')
            publish('/torso_arduino/force_sensors')
            if self.state.torso_arduino_power_external_0 is not None \
            and self.state.torso_arduino_power_external_1 is not None \
            and self.state.torso_arduino_imu_euler_z is not None:
                rospy.loginfo('Force sensors refreshed everything.')
                return
            time.sleep(1)
        rospy.logwarn('Force sensors did not everything!')

    def is_torso_moving(self):
        return self.state.torso_arduino_motor_target_a or self.state.torso_arduino_motor_target_b

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
        self.announce_status('Shutting down.')
        self.halt_motors()
        rospy.loginfo('Shutting down...')
        if self.qr_tracker_process:
            self.qr_tracker_process.stop()
        self.set_pan_freeze(0)
        rospy.loginfo('Done.')

    def start_qr_tracker(self):
        """
        Launches a child node to detect the QR code from the primary camera video feed.
        This code will tell us where the docking station is.
        """
        if getoutput('rosnode list | grep /qr_tracker'):
            # Note, this may be bad. The QR tracker should not be running by default, and we're the only thing that launches it.
            # If we previously crashed, the rosmaster may be incorrectly reporting that /qr_tracker still exists.
            # Unfortunately, killing the node doesn't fix this, since the node is actually already dead.
            # The only solution in this case is to stop and restart master.
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
        rospy.loginfo('qr_tracker.py is_alive: %s', process.is_alive())
        time.sleep(5)

    def qr_code_found(self, t0=0):
        """
        Returns true if we see a QR code matching that of our docking station.
        Returns false otherwise.
        """
        #return self.state.qr_tracker_matches is not None
        return self.state.get_since('qr_tracker_matches', t=t0) is not None

    def is_docked(self):
        """
        Return true if we're docked to a powered docking station. Returns false otherwise.
        """
        return self.state.torso_arduino_power_external_0 and self.state.torso_arduino_power_external_1

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
        return not self.state.torso_arduino_power_external_0 and self.state.torso_arduino_power_external_1

    def center_head(self):
        """
        Causes the head to pan to its center position.
        """
        publish('/head_arduino/tilt_set', 90)
        time.sleep(2)
        publish('/head_arduino/pan_set', 315)
        time.sleep(2)
        publish('/head_arduino/pan_set', 356)
        time.sleep(2)

    def center_torso(self):
        """
        Causes the torso to rotate so it's facing the same direction as the head.
        """

        # Get the head and torso roughly aligned to both point at the QR code.
        for _ in range(10):

            # Center head at QR code.
            self.center_qr_code_using_head()

            # If difference between torso and head is below threshold, then exit.
            head_arduino_pan_degrees = self.state.head_arduino_pan_degrees
            rospy.loginfo('head_arduino_pan_degrees1: %s', head_arduino_pan_degrees)
            head_arduino_pan_degrees = max(head_arduino_pan_degrees, head_arduino_pan_degrees - 360)
            rospy.loginfo('head_arduino_pan_degrees2: %s', head_arduino_pan_degrees)
            if abs(head_arduino_pan_degrees) <= 3:
                break

            # Turn torso part-way (about 90%) towards head. We don't go all the way because this will tend to overshoot.
            print('self.state.head_arduino_pan_degrees0:', self.state.head_arduino_pan_degrees)
            if self.state.head_arduino_pan_degrees < 180:
                deg = self.state.head_arduino_pan_degrees
                rospy.loginfo('Rotating CW: %s', deg)
            else:
                deg = self.state.head_arduino_pan_degrees - 360
                rospy.loginfo('Rotating CCW: %s', deg)
            deg = int(deg*.9)
            rospy.loginfo('Rotating torso by %s degrees.', deg)
            # torso_arduino_imu_euler_z0 = self.state.torso_arduino_imu_euler_z
            self.rotate_torso(deg)
            time.sleep(1)
            # torso_arduino_imu_euler_z1 = self.state.torso_arduino_imu_euler_z
            # torso_z_change = min(
                # abs(torso_arduino_imu_euler_z0 - torso_arduino_imu_euler_z1 - 360),
                # abs(torso_arduino_imu_euler_z0 - torso_arduino_imu_euler_z1 + 360))

            head_deg = -deg
            # print('torso_z_change:', torso_z_change)
            rospy.loginfo('Correcting head angle by %s to compensate for torso rotation.', head_deg)
            self.rotate_head_relative(head_deg)

            # Wait a couple seconds to give the QR tracker time to re-acquire the target.
            time.sleep(2)

            # If we can no longer see the QR code, then that's probably because we overshot, so rotate the head back incrementally until we see it again.
            if not self.state.qr_tracker_matches:
                if head_deg > 0:
                    direction = +1
                else:
                    direction = -1
                self.find_qr_code(direction=direction)

        # Now align head and torso exactly.
        head_deg0 = self.state.head_arduino_pan_degrees
        self.rotate_head_absolute(0)
        time.sleep(1)
        head_deg1 = self.state.head_arduino_pan_degrees

        # Use the torso motors alone to center the QR code.
        head_deg_change = normalize_angle_change(head_deg1, head_deg0)
        if head_deg_change > 0:
            direction = -1
        else:
            direction = +1
        self.center_qr_code_using_torso(direction=direction)

    def find_qr_code(self, direction=1):
        """
        Rotate head until we see a QR code.

        Throws an exception if not code is found after 360 degrees of rotation.

        direction := +1 = clockwise rotation, -1 = counter clockwise rotation
        """
        # self.announce_status('Centering head.')
        # self.center_head()
        self.announce_status('Searching for QR code.')
        # cnt = 0
        slices = 20
        section = 360./slices
        for _ in range(slices):
            # angle = (angle + 18) % 360
            # self.announce_status('Scanning angle %s.' % angle)
            # self.rotate_head_absolute(degrees=angle, threshold=3, timeout=15)
            t0 = time.time()
            self.rotate_head_relative(degrees=section*direction, threshold=3, timeout=15)
            # Wait so our QR processor has time to check our image feed after the pan change.
            # Remember, our camera is low-quality, and recognition is poor in low light and after any kind of movement.
            time.sleep(1)
            if self.qr_code_found(t0=t0):
                self.announce_status('QR code found.')
                return
        self.announce_status('QR code could not be found. Docking terminated.')
        raise Terminate('Unable to find QR code.')

    def center_qr_code_using_head(self):
        """
        Rotates the head until the QR code is roughly in the center of view.
        """
        # Note, our motors don't have good precision, and get stuck at small intervals, so the most we can adjust by are in 10 degree increments.
        angle_of_adjustment = 10
        # Increase angle => rotate CW
        # Decreate angle => rotate CCW
        if not self.state.qr_tracker_matches:
            rospy.logerr('Cannot center QR code. None found.')
            return
        for _ in range(10):
            qr_matches = self.state.qr_tracker_matches
            x = (qr_matches.a[0] + qr_matches.b[0] + qr_matches.c[0] + qr_matches.d[0])/4.
            # y = (self.state.qr_matches.a[1] + self.state.qr_matches.b[1] + self.state.qr_matches.c[1] + self.state.qr_matches.d[1])/4.
            width = qr_matches.width
            # height = qr_matches.height

            # Ideally, this should be 0.5, indicating the QR code is exactly in the center of the horizontal view.
            # In practice, we'll likely never accomplish this, but we'll try to get it within [0.4:0.6]
            position_ratio = x/float(width)
            rospy.loginfo('position_ratio: %s', position_ratio)
            if 0.4 <= position_ratio <= 0.6:
                break
            elif position_ratio < 0.4:
                # QR code is too far left of screen, so move head counter-clockwise to center it.
                new_angle = (self.state.head_arduino_pan_degrees - angle_of_adjustment) % 360
                publish('/head_arduino/pan_set', new_angle)
            elif position_ratio > 0.6:
                # QR code is too far right of screen, so move head clockwise to center it.
                new_angle = (self.state.head_arduino_pan_degrees + angle_of_adjustment) % 360
                publish('/head_arduino/pan_set', new_angle)
            rospy.loginfo('new angle: %s', new_angle)
            time.sleep(1)

    def center_qr_code_using_torso(self, direction=1):
        """
        Centers the QR code in view using only the torso motors to rotate the entire body.
        """
        angle_of_adjustment = 2
        for _ in range(20):

            # Check to see if we have a QR match.
            qr_matches = self.state.qr_tracker_matches
            if qr_matches:
                x = (qr_matches.a[0] + qr_matches.b[0] + qr_matches.c[0] + qr_matches.d[0])/4.
                # y = (self.state.qr_matches.a[1] + self.state.qr_matches.b[1] + self.state.qr_matches.c[1] + self.state.qr_matches.d[1])/4.
                width = qr_matches.width
                # height = qr_matches.height

                # Ideally, this should be 0.5, indicating the QR code is exactly in the center of the horizontal view.
                # In practice, we'll likely never accomplish this, but we'll try to get it within [0.4:0.6]
                position_ratio = x/float(width)
                rospy.loginfo('position_ratio: %s', position_ratio)
                if 0.4 <= position_ratio <= 0.6:
                    rospy.loginfo('Torso centering threshold achieved!')
                    break
                elif position_ratio < 0.4:
                    # QR code is too far left of screen, so move torso counter-clockwise to center it.
                    self.rotate_torso(degrees=-angle_of_adjustment, double_down=True)
                elif position_ratio > 0.6:
                    # QR code is too far right of screen, so move torso clockwise to center it.
                    self.rotate_torso(degrees=+angle_of_adjustment, double_down=True)
                time.sleep(1)

            else:
                # Rotate blindly until we see a QR code.
                rospy.logwarn('No QR code found. Searching blindly.')
                self.rotate_torso(degrees=1*direction)
                # Give QR tracker time to acquire.
                time.sleep(1)

    def announce_status(self, status):
        rospy.loginfo(status)
        if self.audible:
            self.say(re.sub(r'[^a-zA-Z0-9]+', ' ', status))

    def move_torso(self, left_speed, right_speed, milliseconds):
        """
        Move the torso by a certain speed for a certain amount of time.
        """
        assert -255 <= left_speed <= 255
        assert -255 <= right_speed <= 255
        assert milliseconds > 0
        # Tell the state machine to expect a start in motion.
        trigger_up = self.state.set_or_trigger(torso_arduino_motor_target_a=True, torso_arduino_motor_target_b=True)
        # Tell the state machine to expect a drop in motion.
        trigger_down = self.state.set_or_trigger(torso_arduino_motor_target_a=False, torso_arduino_motor_target_b=False, dependencies=[trigger_up])
        # publish('/torso_arduino/motor_speed', [64, 64, 500])
        publish('/torso_arduino/motor_speed', [left_speed, right_speed, milliseconds])
        rospy.loginfo('Waiting for motors to start...')
        trigger_up.wait()
        rospy.loginfo('Waiting for motors to stop...')
        trigger_down.wait()

    def rotate_torso(self, degrees, double_down=False):
        """
        Rotates the torso by the given degrees.

        double_down := If true, and the motors fail to start, increase the degrees and try again.
        """
        degrees = int(degrees)
        if not degrees:
            rospy.logwarn('Attempting to rotate torso by 0 degrees.')
            return
        # imu_euler_z0 = self.state.torso_arduino_imu_euler_z
        # print('torso.imu_euler_z0:', imu_euler_z0)
        for _ in range(10):
            try:
                # Tell the state machine to expect a start in motion.
                trigger_up = self.state.set_or_trigger(torso_arduino_motor_target_a=True, torso_arduino_motor_target_b=True)
                # Tell the state machine to expect a drop in motion.
                trigger_down = self.state.set_or_trigger(torso_arduino_motor_target_a=False, torso_arduino_motor_target_b=False, dependencies=[trigger_up])
                publish('/torso_arduino/motor_rotate', degrees)
                rospy.loginfo('Waiting for rotation to start...')
                trigger_up.wait()
                rospy.loginfo('Waiting for rotation to stop...')
                try:
                    trigger_down.wait(timeout=5)
                except TriggerTimeout:
                    # If we timeout waiting for the motor to stop, that's likely because it already has and even our trigger wasn't fast enough to catch it,
                    # so ignore the timeout.
                    pass
                break
            except TriggerTimeout:
                # A timeout occurred waiting for the motors to start.
                # This either means the degree was too small to cause a change, or the motors have stalled.
                if double_down:
                    degrees *= 2
                    degrees = max(min(degrees, 359), -359)
                else:
                    raise
            except TorsoMotorError as exc:
                rospy.logerr('Error rotating torso: %s', exc)
                traceback.print_exc()

        # FIXME? Disabled, since for large degrees, sometimes causes torso to rotate multiple times if rotation isn't precise enough.
        # Wait until torso stops rotating.
        # self.announce_status('Waiting until we have fully rotated.')
        # first_check = time.time()
        # last_check = time.time()
        # while degrees - abs(self.state.torso_arduino_imu_euler_z - imu_euler_z0) > 5 or time.time() - first_check > 10:
            # if time.time() - last_check > 2:
                # # If we've not moved in N seconds, re-issue the rotate command.
                # publish('/torso_arduino/motor_rotate', degrees)
            # time.sleep(1)
            # rospy.loginfo('Waiting for body to rotate...')
            # print('self.state.imu_euler_z:', self.state.torso_arduino_imu_euler_z)
        # publish('/torso_arduino/motor_rotate', degrees)

    def set_pan_freeze(self, value):
        """
        Ensures the head's pan_freeze flag is set.

        rostopic pub --once /head_arduino/pan_freeze_set std_msgs/Bool 1
        rostopic echo /head_arduino/pan_freeze_get
        """
        if value:
            rospy.loginfo('Ensuring pan angle is set.')
            publish('/head_arduino/pan_set', self.state.head_arduino_pan_degrees)
        for _ in range(10):
            if self.state.head_arduino_pan_freeze_get == value:
                return
            publish('/head_arduino/pan_freeze_set', value)
            rospy.loginfo('Waiting for head_pan_freeze_get to be set to %s.', value)
            time.sleep(0.5)
        raise Exception('Unable to set head_pan_freeze_get to %s.' % value)

    def rotate_head_absolute(self, degrees, threshold=3, timeout=15):
        """
        Rotates the head to an absolute angle.
        """
        for _ in range(timeout):
            print('self.state.head_arduino_pan_degrees:', self.state.head_arduino_pan_degrees)
            if abs(normalize_angle_change(self.state.head_arduino_pan_degrees, degrees)) <= threshold:
                return
            publish('/head_arduino/pan_set', degrees)
            time.sleep(1)
        raise Exception('Unable to pan head to angle %s.' % degrees)

    def rotate_head_relative(self, degrees, threshold=3, timeout=15):
        """
        Rotates the head to an angle relative to the current position.
        """
        deg0 = self.state.head_arduino_pan_degrees
        deg_abs_target = (deg0 + degrees) % 360
        for _ in range(timeout):
            t0 = time.time()
            dist = self.state.head_arduino_pan_degrees - deg0
            if degrees < 0:
                # Turning CCW.
                if dist > 180:
                    dist -= 360
            else:
                # Turning CW.
                if dist < -180:
                    dist += 360
            change = abs(dist - degrees)
            if change <= threshold:
                return
            publish('/head_arduino/pan_set', deg_abs_target)
            # Wait until we get an updated pan position.
            self.state.get_since_until('head_arduino_pan_degrees', t=t0)
        raise Exception('Unable to rotate head by %s degrees.' % degrees)

    def undock(self):
        publish('/torso_arduino/undock') # std_msgs/Empty

    def move_forward_until_aligned(self, threshold=3):
        """
        Drives forwards, with the QR code roughly perpendicular, in short bursts until the QR code is roughly seen straight on.

        We need to drive in bursts since the camera is low-quality and can only capture a clear enough image to do QR recognition when the platform is still.

        threshold := The number of degrees plus/minus around 0 to be before stopping.
        """
        deflection_angle_degrees0 = None
        while 1:

            # self.announce_status('Un-freezing head angle.')
            self.set_pan_freeze(0)

            # self.announce_status('Centering QR code in view.')
            self.center_qr_code_using_head()

            # See if we still have a match, timing out after N tries.
            # This assumes we've already locked our vision onto the QR code and have frozen the pan angle.
            # Note, due to our movement, the pan freeze may take a couple seconds to update after we stop,
            # so the QR code match may be temporarily lost. We wait to re-establish the lock.
            qr_tracker_matches = None
            for _ in range(10):
                qr_tracker_matches = self.state.qr_tracker_matches
                if qr_tracker_matches:
                    break
                rospy.loginfo('Waiting for QR match...')
                time.sleep(1)
            if not qr_tracker_matches:
                rospy.logerr('QR match lost.')
                raise Terminate('QR match lost.')

            # Get the deflection angle between the QR codes orthogonal line and our line of sight. Also known as theta1.
            deflection_angle_degrees = qr_tracker_matches.deflection_angle * 180. / pi
            rospy.loginfo('deflection_angle_degrees: %s', deflection_angle_degrees)
            if deflection_angle_degrees0 is None:
                deflection_angle_degrees0 = deflection_angle_degrees
            rospy.loginfo('QR deflection angle: %s', deflection_angle_degrees)
            if abs(deflection_angle_degrees) <= threshold:
                # Stop once we're looking almost straight-on.
                rospy.loginfo('Deflection angle achieved.')
                return
            if (deflection_angle_degrees0 < 0 and deflection_angle_degrees > 0) or (deflection_angle_degrees0 > 0 and deflection_angle_degrees < 0):
                # Stop if our deflection angle started out with one sign, and flipped to the other, signifying that we overshot our target threshold.
                rospy.loginfo('Deflection angle overshot.')
                return

            # Drive forward at quarter speed (64/256.) for half a second.
            self.move_torso(left_speed=64, right_speed=64, milliseconds=500)

    def move_forward_until_docked(self):
        """
        Drives forward, attempting to keep the QR code match acquired and centered in view.

        Assumes head and torso alignment are locked and torso is already centered on QR code.
        """
        last_jitter = True
        while 1:

            if self.is_docked():
                rospy.loginfo('Docking achieved!')
                return

            # Realign ourselves using the QR code.
            qr_tracker_matches = self.state.qr_tracker_matches
            if not qr_tracker_matches:
                rospy.logerr('QR code match lost!')
                # If we've lost the QR code, then do the jitterbug, hoping we're close enough to connect.
                if last_jitter:
                    self.move_torso(left_speed=64, right_speed=32, milliseconds=250)
                else:
                    self.move_torso(left_speed=32, right_speed=64, milliseconds=250)
                last_jitter = not last_jitter
            else:
                # If we still see the QR code, the realign so we can drive straight.
                self.center_qr_code_using_torso()

            # Drive forward at quarter speed (64/256.) for half a second.
            try:
                self.move_torso(left_speed=64, right_speed=64, milliseconds=1000)
            except TriggerTimeout:
                # If we dock during our approach movement, then the motor controller's safety protocol will cutoff the motors
                # and not give us a proper transition, leading to a trigger timeout.
                # In this case, we can ignore it.
                pass

            # Give the QR tracker time to catch up.
            time.sleep(1)

    def dock_iter(self):
        """
        This is the main docking procedure.
        This will stop iterating once the docking procedure has succeeded or reached a failure condition.
        """
        t0 = time.time()
        try:
            self.announce_status('Beginning docking procedure.')

            # self.announce_status('Checking current state.')
            # if self.is_docked():
                # self.announce_status('We are already docked. Nothing to do.')
                # return

            # if self.is_dead_docked():
                # self.announce_status('We sense the docking magnet, but no power, possibly from a previous failed docking attempt.')
                # self.announce_status('Undocking to abort last attempt and retry.')
                # self.undock()
                # yield
                # time.sleep(5)
                # yield

            if not self.qr_code_found():
                raise Terminate('QR code not found.')#TODO:remove
                # self.find_qr_code()

            self.rotate_head_absolute(0)
            self.center_qr_code_using_torso()
            self.move_forward_until_docked()
            return

            # self.announce_status('Centering QR code in view.')
            # self.center_qr_code_using_head()
            # yield
            # self.announce_status('View centered.')

            # self.announce_status('Freezing head angle.')
            # self.set_pan_freeze(1)
            # yield

            # self.announce_status('Angling body to be perpendicular to QR code.')
            # # Calculate torso angle rotation needed so that the front faces inward by 90 degrees to the head position.
            # # Note, a positive degree proceeds CW when looking down.
            # # e.g. if we're facing -10 degrees to the left == 350 degrees, then we need to rotate the torso by -10 degrees to center,
            # # and then another -90 degrees to be perpendicular.
            # #TODO:estimate angle of offset of QR code
            # qr_tracker_matches = None
            # while 1:
                # if self.state.qr_tracker_matches:
                    # qr_tracker_matches = self.state.qr_tracker_matches
                    # break
                # rospy.loginfo('Waiting for QR match...')
                # time.sleep(1)
            # # The head's pan angle is the measure of deflection between the head's center and the torso's center.
            # theta1 = self.state.head_arduino_pan_degrees
            # # Get the deflection angle between the QR codes orthogonal line and our line of sight. Also known as theta1.
            # deflection_angle_degrees = qr_tracker_matches.deflection_angle * 180. / pi
            # print('deflection_angle_degrees:', deflection_angle_degrees)
            # # Estimate the angle between our head's angle and the angle to turn our head completely perpendicular to the QR code. Also known as theta2.
            # # Note that theta2 + theta3 + 90 for all the angle in a right triangle between the camera, the QR code and our ideal docking start position.
            # theta2 = 180 - (90 + deflection_angle_degrees)
            # print('theta1 (head pan_degrees):', theta1)
            # print('theta2 (angle between head and docking start point):', theta2)
            # if theta1 > 180:
                # # Need to rotate CCW.
                # torso_rotation = (theta1 - 360) - theta2
                # print('torso_rotation.a:', torso_rotation)
            # else:
                # # Need to rotate CW.
                # torso_rotation = theta1 - theta2
                # print('torso_rotation.b:', torso_rotation)
            # rospy.loginfo('torso_rotation: %s', torso_rotation)
            # self.announce_status('Rotating torso by %s.' % int(torso_rotation))
            # self.rotate_torso(torso_rotation)
            # self.announce_status('Body angled.')

            # self.announce_status('Moving body to be infront of QR code.')
            # self.move_forward_until_aligned()

            #TODO
            # self.announce_status('Freezing head angle.')
            # self.set_pan_freeze(1)
            # self.announce_status('Rotating torso to face dock.')
            # self.center_torso()

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
        except KeyboardInterrupt:
            self.announce_status('Emergency interrupt received.')
        except Terminate as exc:
            self.announce_status('Docking procedure terminated. %s' % exc)
            traceback.print_exc()
        except Exception as exc:
            self.announce_status('An unexpected error occurred. %s' % exc)
            traceback.print_exc()
        else:
            self.announce_status('Docking procedure complete.')
        finally:
            self.on_shutdown()

if __name__ == '__main__':
    #TODO:Fix? Conflicts with roslaunch parameters?
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--audible', default=False, action='store_true', help='If given audible status updates will be given.')
    # parser.add_argument('__name', default='dock_node')
    # parser.add_argument('__log', default='dock_node')
    # args = parser.parse_args()
    # DockNode(**args.__dict__)
    import sys
    audible = '--audible' in sys.argv
    DockNode(audible=audible)
