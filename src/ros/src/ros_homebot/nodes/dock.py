#!/usr/bin/env python
from functools import partial
import importlib
import time
from commands import getoutput

import rospy
import roslaunch

from ros_qr_tracker.srv import SetTarget
from ros_qr_tracker.msg import Percept

class State(object):
    pass

topic_definitions = {
    '/head_arduino/tilt/set': 'std_msgs/Int16',
    '/head_arduino/pan/set': 'std_msgs/Int16',
}

def get_published_topics():
    topic_types = topic_definitions.copy()
    topic_types.update(dict(rospy.get_published_topics()))
    # print('topic_types:', topic_types)
    raw_topics = getoutput('rostopic list').split('\n')
    for raw_topic in raw_topics:
        if raw_topic not in topic_types:
            topic_types[raw_topic] = 'std_msgs/Empty'
    return topic_types

def subscribe(topic, cb):
    """
    Shortcut that dynamically looks up the subscriber for a topic, as well as the type class, and subscribes.
    """
    topic_types = get_published_topics()
    assert topic in topic_types, 'Topic "%s" is not one of the valid published topics: %s' % (topic, ', '.join(sorted(topic_types)))
    typ = topic_types[topic]
    typ_module_name, typ_cls_name = typ.split('/')
    typ_cls = getattr(importlib.import_module('%s.msg' % typ_module_name), typ_cls_name)
    rospy.loginfo('subscribing to: %s %s %s' % (topic, typ_cls, cb))
    return rospy.Subscriber(topic, typ_cls, cb)

def publish(topic, data=None):
    """
    Shortcut that dynamically looks up the publisher for a topic, as well as the type class, wraps the data in the class, and then publishes it.
    """
    topic_types = get_published_topics()
    assert topic in topic_types, 'Topic "%s" is not one of the valid published topics: %s' % (topic, ', '.join(sorted(topic_types)))
    typ = topic_types[topic]
    print('topic:', topic)
    print('typ:', typ)
    typ_module_name, typ_cls_name = typ.split('/')
    typ_cls = getattr(importlib.import_module('%s.msg' % typ_module_name), typ_cls_name)
    pub = rospy.Publisher(topic, typ_cls, queue_size=1)
    if typ == 'std_msgs/Empty':
        data = typ_cls()
    else:
        data = typ_cls(data)
    rospy.loginfo('publishing %s to %s' % (data, topic))
    pub.publish(data)

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

    def __init__(self):
        rospy.init_node('dock', log_level=rospy.DEBUG)

        self.state = State()
        self.state.index = self.SEARCHING
        self.state.ep0 = None
        self.state.ep1 = None
        self.state.pan_degrees = None
        self.state.qr_matches = None

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

        # Track the position of the head's pan degree.
        subscribe('/head_arduino/pan/degrees', partial(self._set_state, name='pan_degrees'))

        # Track QR matches.
        subscribe('/qr_tracker/matches', partial(self._set_state, name='qr_matches'))

        self.force_sensors()

        dock_iter = self.dock_iter()
        while not rospy.is_shutdown():
            # time.sleep(1)
            try:
                dock_iter.next()
            except StopIteration:
                break

    def on_shutdown(self):
        rospy.loginfo('Shutting down...')
        if self.qr_tracker_process:
            self.qr_tracker_process.stop()
        rospy.loginfo('Done.')

    def start_qr_tracker(self):
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
        data = msg.data
        if isinstance(msg, Percept):
            data = msg
        rospy.loginfo('setting state: %s=%s(type=%s)' % (name, data, type(msg)))
        assert hasattr(self.state, name)
        setattr(self.state, name, data)
        setattr(self.state, name+'_update', time.time())

    def force_sensors(self):
        # publish('/head_arduino/force_sensors')
        for _ in range(10):
            publish('/torso_arduino/force_sensors')
            if self.state.ep0 is not None and self.state.ep1 is not None:
                break
            time.sleep(1)

    def center_head(self):
        publish('/head_arduino/tilt/set', 90)
        time.sleep(2)
        publish('/head_arduino/pan/set', 315)
        time.sleep(2)
        publish('/head_arduino/pan/set', 356)
        time.sleep(2)

    def qr_code_found(self):
        return self.state.qr_matches is not None

    def is_dock_connected(self):
        return self.state.ep0 and self.state.ep0

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

    def dock_iter(self):
        rospy.loginfo('Beginning dock procedure.')

        rospy.loginfo('Step 1: Centering the head.')
        self.center_head()
        yield

        rospy.loginfo('Step 2: Searching for QR code.')
        angle = 356
        cnt = 0
        for _ in range(20):
            angle = (angle + 18) % 360
            rospy.loginfo('Scanning angle %s.' % angle)
            publish('/head_arduino/pan/set', angle)
            time.sleep(1)
            if self.qr_code_found():
                rospy.loginfo('QR code found.')
                break
        if not self.qr_code_found():
            rospy.loginfo('QR code could not be found. Procedure terminated.')
            return

        rospy.loginfo('Centering QR code in view.')
        self.center_qr_in_view()
        rospy.loginfo('View centered.')

        rospy.loginfo('Angling body perpendicular.')

        rospy.loginfo('Body angled.')

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
        rospy.loginfo('Dock procedure terminated.')

    def undock(self):
        pass

if __name__ == '__main__':
    DockNode()
