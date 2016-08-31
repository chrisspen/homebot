
from django import template

from ros_homebot_python import constants as c
from ros_homebot_python.node import get_name, get_socket_event_name_in

register = template.Library()

@register.simple_tag
def socketio_event_name(device, packet_id):
    assert device in c.NAME_TO_INDEX, 'Invalid device: %s' % device
    assert packet_id in c.ALL_IDS, 'Invalid packet: %s' % packet_id
    return '%s_%s' % (device.lower(), get_name(packet_id))

@register.simple_tag
def socketio_event_name_in(device, packet_id):
    assert device in c.NAME_TO_INDEX, 'Invalid device: %s' % device
    assert packet_id in c.ALL_IDS, 'Invalid packet: %s' % packet_id
    return get_socket_event_name_in(device, packet_id)
