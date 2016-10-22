import time
from threading import Thread, RLock
from Queue import Queue

from socketio.namespace import BaseNamespace
from socketio.mixins import BroadcastMixin
from socketio.sdjango import namespace

import rospy
import std_msgs.msg
import std_srvs.srv

from ros_homebot_msgs import msg as msgs
from ros_homebot_python import constants as c
from ros_homebot_python.node import get_packet_id

thread = None
running = True
connections = 0
incoming_queue = None
_broadcaster = None
node = None

def get_broadcaster():
    return _broadcaster

def get_connections():
    return connections

@namespace('/echo')
class EchoNamespace(BaseNamespace, BroadcastMixin):
    
    def recv_connect(self):
        global thread, connections, _broadcaster
        
        def background_thread():
            """Example of how to send server generated events to clients."""
            count = 0
            while running:
                time.sleep(1)
                count += 1
                self.broadcast_event('update_count', {'count': count})
                
        connections += 1
        if thread is None:
            _broadcaster = self
            thread = Thread(target=background_thread)
            thread.daemon = True
            thread.start()
    
    def recv_disconnect(self):
        global connections
        connections -= 1
    
    def call_method(self, method_name, packet, *args):
        print 'call_method:', method_name, packet, args
        
        if hasattr(self, method_name):
            # Use override method.
            return super(EchoNamespace, self).call_method(method_name, packet, *args)
        elif node:
            # Otherwise pass through the event.
            parts = method_name.split('_')
            assert parts[0] == 'on'
            device = parts[1].upper()
            if device in (c.HEAD, c.TORSO):
                packet_name = '_'.join(parts[2:])
                packet_id = get_packet_id(packet_name)
                node.publish_packet_write(device, packet_id, *args)
    
    def on_head_reset(self, state=None):
        print 'head reset'
        rospy.ServiceProxy('/head_arduino/reset', std_srvs.srv.Empty)()
    
    def on_torso_reset(self, state=None):
        print 'torso reset'
        rospy.ServiceProxy('/torso_arduino/reset', std_srvs.srv.Empty)()
    
    def on_torso_shutdown(self, state=None):
        print 'shutdown:', state
        if node:
            node.publish_packet_write(c.NAME_TORSO, c.ID_SHUTDOWN, state)
    
#     def on_msg(self, msg):
#         print '1'*80
#         print 'msg:', msg
#         pkt = dict(type='event',
#                    name='msg',
#                    args='Someone said: {0}'.format(msg),
#                    endpoint=self.ns_name)
# 
#         for sessid, socket in self.socket.server.sockets.iteritems():
#             socket.send_packet(pkt)
