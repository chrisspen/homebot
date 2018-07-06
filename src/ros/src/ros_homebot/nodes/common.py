import importlib
import time
from commands import getoutput

import yaml

import rospy

_publisher_cache = {} # {topic: publisher_instance}

def get_published_topics(target=None):
    """
    Retrieves a hash of topics of the form {topic: type}.
    """
    topic_types = {}
    if target:
        raw_topics = getoutput('rostopic list %s --verbose' % target).split('\n')
    else:
        raw_topics = getoutput('rostopic list --verbose').split('\n')
    for raw_topic in raw_topics:
        parts = raw_topic.split(' ')
        if len(parts) == 6 and parts[1] == '*':
            _, _, topic, type_name, cnt, io_type = parts
            type_name = type_name[1:-1]
            topic_types[topic] = type_name
    return topic_types


class TopicWaitTimeout(Exception):
    pass


def wait_for_topic(topic, timeout=10):
    """
    Blocks until the given topic is listed by the `rostopic list`.

    Timeout is in seconds.
    """
    t0 = time.time()
    while 1:
        if getoutput('rostopic list | grep %s' % topic):
            return
        if timeout and time.time() - t0 >= timeout:
            raise TopicWaitTimeout


class NodeWaitTimeout(Exception):
    pass


def wait_for_node(node, timeout=10):
    """
    Blocks until the given node is listed by the `rosnode list`.

    Timeout is in seconds.
    """
    t0 = time.time()
    while 1:
        if getoutput('rosnode list | grep %s' % node):
            return
        if timeout and time.time() - t0 >= timeout:
            raise NodeWaitTimeout


def has_camera():
    """
    Returns true if a supported physical camera is connected to the localhost.
    """
    ret = getoutput('vcgencmd get_camera') or ''
    return 'supported=1' in ret


def subscribe(topic, cb):
    """
    Dynamically looks up the subscriber for a topic, as well as the type class, and subscribes.
    """
    topic_types = get_published_topics(topic)
    if topic not in topic_types:
        all_topic_types = get_published_topics()
        raise Exception('Topic "%s" is not one of the valid published topics: %s' % (topic, ', '.join(sorted(all_topic_types))))
    typ = topic_types[topic]
    typ_module_name, typ_cls_name = typ.split('/')
    typ_cls = getattr(importlib.import_module('%s.msg' % typ_module_name), typ_cls_name)
    rospy.loginfo('subscribing to: %s %s %s', topic, typ_cls, cb)
    return rospy.Subscriber(topic, typ_cls, cb)

def publish(topic, data=None, queue_size=10, init_seconds=5):
    """
    Dynamically looks up the publisher for a topic, as well as the type class, wraps the data in the class, and then publishes it.
    """
    topic_types = get_published_topics(topic)
    if topic not in topic_types:
        all_topic_types = get_published_topics()
        raise Exception('Topic "%s" is not one of the valid published topics: %s' % (topic, ', '.join(sorted(all_topic_types))))
    typ = topic_types[topic]
    typ_module_name, typ_cls_name = typ.split('/')
    typ_cls = getattr(importlib.import_module('%s.msg' % typ_module_name), typ_cls_name)
    if topic not in _publisher_cache:
        pub = _publisher_cache[topic] = rospy.Publisher(topic, typ_cls, queue_size=queue_size)
        # When first creating the publisher, wait for a maximum of N seconds or when a subscriber appears, so our first message doesn't get lost in the ether.
        t0 = time.time()
        while not pub.get_num_connections() and time.time() - t0 <= init_seconds:
            rospy.loginfo("Waiting for subscriber to connect.")
            rospy.sleep(1)
    pub = _publisher_cache[topic]
    if typ == 'std_msgs/Empty':
        data = typ_cls()
    elif hasattr(typ_cls, 'layout') and hasattr(typ_cls, 'data'):
        # MultiArray types have both layout and data attributes, but layout is optional and will be implied by the data list.
        # However, we must explicitly specify the attribute name, otherwise the type will complain about mismatched types.
        if isinstance(data, basestring):
            data = yaml.load(data)
        assert isinstance(data, (dict, list)), 'MultiArray type data must be a list, not %s.' % type(data)
        if isinstance(data, list):
            data = typ_cls(data=data)
        else:
            data = typ_cls(**data)
    else:
        data = typ_cls(data)
    rospy.loginfo('publishing %s to %s', data, topic)
    pub.publish(data)

if __name__ == '__main__':
    import sys
    action = sys.argv[1]
    if action == 'pub':
        #TODO:fix? This can't connect to rosmaster?
        rospy.init_node('common_wrapper')
        time.sleep(2)
        publish(sys.argv[2], sys.argv[3])
    else:
        raise NotImplementedError('Unknown action: %s' % action)
