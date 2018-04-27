import importlib
from commands import getoutput

import rospy

topic_definitions = {
    '/head_arduino/tilt/set': 'std_msgs/Int16',
    '/head_arduino/pan/set': 'std_msgs/Int16',
}

_publisher_cache = {} # {topic: publisher_instance}

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
    if topic not in _publisher_cache:
        _publisher_cache[topic] = rospy.Publisher(topic, typ_cls, queue_size=1)
    pub = _publisher_cache[topic]
    if typ == 'std_msgs/Empty':
        data = typ_cls()
    else:
        data = typ_cls(data)
    rospy.loginfo('publishing %s to %s' % (data, topic))
    pub.publish(data)
