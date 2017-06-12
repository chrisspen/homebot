"""
2016.8.28 CKS
Experimental attempt to train a classifier to estimate depth from pairs of images
and laser scan readings.

References:

    http://www.cs.cornell.edu/~asaxena/learningdepth/

    http://www.cs.cornell.edu/~asaxena/learningdepth/ijcv_monocular3dreconstruction.pdf
    hierarchical, multiscale Markov Random Field

    https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images.pdf
    discriminatively-trained Markov Random Field

    https://en.wikipedia.org/wiki/Markov_random_field
"""

#TODO: collect image+line+laser sets and dump to filesystem
import os
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as PilImage
import rosbag

BAG_FN = '../bags/lrf_sample.bag'

DATA_DIR = '../data/lrf_sample'

CAMERA_TOPIC = '/homebot_lrf/image/off'
LINE_TOPIC = '/homebot_lrf/line/image'
SCAN_TOPIC = '/homebot_lrf/scan'

bridge = CvBridge()

def iter_bag_serially(fn):
    last_camera_msg = None
    last_line_msg = None
    last_scan_msg = None

    #rosbag record -O lrf_sample /raspicam/image/compressed /homebot_lrf/line/image /homebot_lrf/scan
    bag = rosbag.Bag(fn)
    for topic, msg, t in bag.read_messages():#topics=['chatter', 'numbers']):
#         if not topic.startswith('/homebot_lrf'):
#             continue
#         print 'topic:', topic
#         print 'msg:', type(msg)
#         print 't:', t

        # Collect messages.
        if topic == CAMERA_TOPIC:
            last_camera_msg = msg
        elif topic == LINE_TOPIC:
            last_line_msg = msg
        elif topic == SCAN_TOPIC:
            last_scan_msg = msg

        # Yield once we have a complete set, and then reset.
        if last_camera_msg and last_line_msg and last_scan_msg:
            yield last_camera_msg, last_line_msg, last_scan_msg
            last_camera_msg = None
            last_line_msg = None
            last_scan_msg = None

    bag.close()

i = 0
for camera_msg, line_msg, scan_msg in iter_bag_serially(BAG_FN):
    i += 1

    local_dir = os.path.join(DATA_DIR, str('%05i' % i))
    if not os.path.isdir(local_dir):
        os.mkdir(local_dir)

#     print camera_msg, lime_msg, scan_msg
    print 'i:', i

    assert camera_msg.format == 'jpg', 'Invalid format: %s' % camera_msg.format
    open(os.path.join(local_dir, 'raw.jpg'), 'wb').write(str(camera_msg.data))

    open(os.path.join(local_dir, 'scan.yaml'), 'w').write(str(scan_msg))
#     print scan_msg

    #http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html
#     print type(line_msg), line_msg.encoding, len(line_msg.data), len(line_msg.data[0])
    cv_image = bridge.imgmsg_to_cv2(line_msg, "bgra8")
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
    pil_image = PilImage.fromarray(cv_image)
    pil_image.save(os.path.join(local_dir, 'line.bmp'))
    #break

