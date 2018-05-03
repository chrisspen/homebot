import importlib
from commands import getoutput

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

def publish(topic, data=None, queue_size=1):
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
        _publisher_cache[topic] = rospy.Publisher(topic, typ_cls, queue_size=queue_size)
    pub = _publisher_cache[topic]
    if typ == 'std_msgs/Empty':
        data = typ_cls()
    else:
        data = typ_cls(data)
    rospy.loginfo('publishing %s to %s', data, topic)
    pub.publish(data)
