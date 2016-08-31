#!/usr/bin/env python

from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import StringIO
import time
from Queue import Queue, Full, Empty

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

incoming_data = Queue(maxsize=1)
running = True
node = None
capture = False

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global capture
        if not running:
            return
        elif self.path.endswith('.mjpg'):
            # Stream the mjpeg data.
            capture = True
            try:
                self.send_response(200)
                self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
                self.end_headers()
                while running:
                    try:
                        #msg = incoming_data.get()
                        msg = rospy.wait_for_message('/raspicam/image/compressed', CompressedImage)
                        self.wfile.write("--jpgboundary")
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(msg.data)))
                        self.end_headers()
                        self.wfile.write(msg.data)
                        #time.sleep(0.05)
                    except Empty:
                        #print 'empty processor'
                        time.sleep(0.1)
                        continue
                    except KeyboardInterrupt:
                        break
            finally:
                capture = False
            return
        else:#if self.path.endswith('.html'):
            # Render simple viewer.
            host = self.headers.get('Host')
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            self.wfile.write('<img src="http://%s/cam.mjpg"/>' % host)
            self.wfile.write('</body></html>')
            return
            
class VideoStreamingNode(object):
    
    def __init__(self):
        global node
        node = self
        try:
            rospy.init_node('homebot_mjpeg', log_level=rospy.DEBUG)
            #rospy.Subscriber("/raspicam/image/compressed", CompressedImage, self.handle_compressedimage)
            rospy.on_shutdown(self.shutdown)
            
            self.port = int(rospy.get_param('~port', 8181))
            
            self.server = HTTPServer(('', self.port), CamHandler)
            print 'Server is listening on port %i.' % self.server.server_port
            while not rospy.is_shutdown():
                self.server.handle_request()
        except KeyboardInterrupt:
            self.server.socket.close()

#     def handle_compressedimage(self, msg):
#         global incoming_data
#     #     print 'received image:', msg.header
#         if not capture:
#             return
#         while running:
#             try:
#                 incoming_data.put(msg)
#                 break
#             except Full:
#                 print 'emptying...'
#                 incoming_data.get()

    def shutdown(self):
        global running
        print 'Shutting down...'
        running = False
        self.server.socket.close()
        print 'Shutdown.'

if __name__ == '__main__':
    VideoStreamingNode()
