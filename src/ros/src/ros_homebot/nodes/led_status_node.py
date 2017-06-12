#!/usr/bin/env python
from __future__ import print_function

from diagnostic_msgs.msg import DiagnosticStatus

from ros_homebot_python.robot import Robot

class LEDStatusNode(Robot):
    """
    Sets the status LED to reflect system state.

    Colors:

        flashing green = system starting up
        green = ok
        orange = warning
        red = error

    Inspect the diagnostics toplevel status with:

        rostopic echo /diagnostics_toplevel_state
    """

    node_name = 'led_status_node'

    def init_subscriptions(self):
        self.subscribe('/diagnostics_toplevel_state', DiagnosticStatus, self.on_diagnostics_toplevel_state)

    def on_shutdown(self):
        self.set_rgbled((0, 0, 0))

    def on_diagnostics_toplevel_state(self, msg):
        if not self.is_shutdown() and msg.name == 'toplevel_state':
            if msg.level in (DiagnosticStatus.OK,):
                self.set_rgbled((0, 254, 0))
            elif msg.level in (DiagnosticStatus.WARN, DiagnosticStatus.STALE):
                self.set_rgbled((254, 127, 0))
            elif msg.level in (DiagnosticStatus.ERROR,):
                self.set_rgbled((254, 0, 0))

if __name__ == '__main__':
    LEDStatusNode()
