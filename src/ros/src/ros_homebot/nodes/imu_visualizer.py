#! /usr/bin/env python
"""
Renders a 3d model showing the IMU readings.

    sudo apt-get install libglfw3*
    pip install vispy

"""
from __future__ import print_function

from math import pi

import rospy
from sensor_msgs.msg import Imu
import tf
import numpy as np
from vispy import app, gloo
from vispy.util.transforms import perspective, translate, rotate

# from ros_homebot_python import constants as c
# from ros_homebot_python.utils import assert_node_alive

vert = """
// Uniforms
// ------------------------------------
uniform   mat4 u_model;
uniform   mat4 u_view;
uniform   mat4 u_projection;
uniform   vec4 u_color;

// Attributes
// ------------------------------------
attribute vec3 a_position;
attribute vec4 a_color;
attribute vec3 a_normal;

// Varying
// ------------------------------------
varying vec4 v_color;

void main()
{
    v_color = a_color * u_color;
    gl_Position = u_projection * u_view * u_model * vec4(a_position,1.0);
}
"""


frag = """
// Varying
// ------------------------------------
varying vec4 v_color;

void main()
{
    gl_FragColor = v_color;
}
"""

# -----------------------------------------------------------------------------
def cube():
    """
    Build vertices for a colored cube.

    V  is the vertices
    I1 is the indices for a filled cube (use with GL_TRIANGLES)
    I2 is the indices for an outline cube (use with GL_LINES)
    """
    vtype = [('a_position', np.float32, 3),
             ('a_normal', np.float32, 3),
             ('a_color', np.float32, 4)]
    # Vertices positions
    v = [[1, 1, 1], [-1, 1, 1], [-1, -1, 1], [1, -1, 1],
         [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, -1]]
    # Face Normals
    n = [[0, 0, 1], [1, 0, 0], [0, 1, 0],
         [-1, 0, 1], [0, -1, 0], [0, 0, -1]]
    # Vertice colors
    colors = [[0, 1, 1, 1], [0, 0, 1, 1], [0, 0, 0, 1], [0, 1, 0, 1],
         [1, 1, 0, 1], [1, 1, 1, 1], [1, 0, 1, 1], [1, 0, 0, 1]]

    V = np.array([(v[0], n[0], colors[0]), (v[1], n[0], colors[1]),
                  (v[2], n[0], colors[2]), (v[3], n[0], colors[3]),
                  (v[0], n[1], colors[0]), (v[3], n[1], colors[3]),
                  (v[4], n[1], colors[4]), (v[5], n[1], colors[5]),
                  (v[0], n[2], colors[0]), (v[5], n[2], colors[5]),
                  (v[6], n[2], colors[6]), (v[1], n[2], colors[1]),
                  (v[1], n[3], colors[1]), (v[6], n[3], colors[6]),
                  (v[7], n[3], colors[7]), (v[2], n[3], colors[2]),
                  (v[7], n[4], colors[7]), (v[4], n[4], colors[4]),
                  (v[3], n[4], colors[3]), (v[2], n[4], colors[2]),
                  (v[4], n[5], colors[4]), (v[7], n[5], colors[7]),
                  (v[6], n[5], colors[6]), (v[5], n[5], colors[5])],
                 dtype=vtype)
    I1 = np.resize(np.array([0, 1, 2, 0, 2, 3], dtype=np.uint32), 6 * (2 * 3))
    I1 += np.repeat(4 * np.arange(2 * 3, dtype=np.uint32), 6)

    I2 = np.resize(
        np.array([0, 1, 1, 2, 2, 3, 3, 0], dtype=np.uint32), 6 * (2 * 4))
    I2 += np.repeat(4 * np.arange(6, dtype=np.uint32), 8)

    return V, I1, I2


# -----------------------------------------------------------------------------
class IMUVisualizer(app.Canvas):

    def __init__(self):
        app.Canvas.__init__(self, keys='interactive', size=(800, 600))

        rospy.init_node('imu_visualizer')
#         assert_node_alive('torso_arduino')

        rospy.Subscriber('/imu_data', Imu, self.on_imu)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.vertices, self.filled, self.outline = cube()
        self.filled_buf = gloo.IndexBuffer(self.filled)
        self.outline_buf = gloo.IndexBuffer(self.outline)

        self.program = gloo.Program(vert, frag)
        self.program.bind(gloo.VertexBuffer(self.vertices))

        self.view = translate((0, 0, -5))
        self.model = np.eye(4, dtype=np.float32)

        gloo.set_viewport(0, 0, self.physical_size[0], self.physical_size[1])
        self.projection = perspective(45.0, self.size[0] /
                                      float(self.size[1]), 2.0, 10.0)

        self.program['u_projection'] = self.projection
        self.program['u_model'] = self.model
        self.program['u_view'] = self.view

        self.yaw = 0 # yaw
        self.roll = 0 # roll
        self.pitch = 0 # pitch

        gloo.set_clear_color('white')
#         gloo.set_clear_color('black')
        gloo.set_state('opaque')
        gloo.set_polygon_offset(1, 1)

        self.first = True
        self.yaw0 = None

        self._timer = app.Timer('auto', connect=self.on_timer, start=True)

        self.show()

        app.run()

        rospy.signal_shutdown("Shutting done.")
        self.shutdown()
        print('Node shutdown.')

    def on_imu(self, msg):
#         print('imu:', msg)

        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
#         print('quaternion:', quaternion)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = -euler[0]
        roll = -euler[1]
        pitch = euler[2]

        # ROS uses radians, but vispy uses degrees.
        roll = roll*180/pi
        pitch = pitch*180/pi
        yaw = yaw*180/pi

        # The yaw is absolute but may be centered to an arbitrary position,
        # so to use the current position as center, we record the initial yaw
        # and zero this out.
        if self.first:
            self.first = False
            self.yaw0 = yaw
        yaw -= self.yaw0
        yaw = yaw % 360

#         print('roll:', roll, 'pitch:', pitch, 'yaw:', yaw)
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch

    # ---------------------------------
    def on_timer(self, event):

#         self.yaw += .5 # yaw
#         self.roll += .5 # roll
#         self.pitch += .5 # pitch

        self.model = np.dot(
            rotate(self.pitch, (1, 0, 0)),
            np.dot(
                rotate(self.yaw, (0, 1, 0)),
                rotate(self.roll, (0, 0, 1)),
            ),
        )
        self.program['u_model'] = self.model
        self.update()

    # ---------------------------------
    def on_resize(self, event):
        gloo.set_viewport(0, 0, event.physical_size[0], event.physical_size[1])
        self.projection = perspective(45.0, event.size[0] /
                                      float(event.size[1]), 2.0, 10.0)
        self.program['u_projection'] = self.projection

    # ---------------------------------
    def on_draw(self, event):
        gloo.clear()

        # Filled cube

        gloo.set_state(blend=False, depth_test=True, polygon_offset_fill=True)
        self.program['u_color'] = 1, 1, 1, 1
        self.program.draw('triangles', self.filled_buf)

        # Outline
        gloo.set_state(blend=True, depth_test=True, polygon_offset_fill=False)
        gloo.set_depth_mask(False)
        self.program['u_color'] = 0, 0, 0, 1
        self.program.draw('lines', self.outline_buf)
        gloo.set_depth_mask(True)

    def shutdown(self):
        rospy.loginfo('Shutting down...')
        self._timer.stop()
        rospy.loginfo('Done.')

if __name__ == '__main__':
    IMUVisualizer()
