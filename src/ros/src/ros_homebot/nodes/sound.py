#! /usr/bin/env python
import os
import sys
import traceback
from threading import RLock

# import roslib
#roslib.load_manifest('ros_homebot')
import rospy
import actionlib
import std_srvs.srv

import ros_homebot_msgs.srv
import ros_homebot.msg
from ros_homebot_python import constants as c
# from ros_homebot_python import utils

VOICE1 = 'en+f3'
VOICE2 = 'english-mb-en1'
VOICES = (
    VOICE1,
)

class SoundServer:
    """
    Manages text-to-speech and tone generation.
    """

    def __init__(self, name):

        self.running = True

        self._lock = RLock()

        self._last_volume = None

        rospy.on_shutdown(self.shutdown)

        rospy.Service(
                '~say',
                ros_homebot_msgs.srv.TTS,
                self.say)

        rospy.Service(
                '~shutup',
                std_srvs.srv.Empty,
                self.say)

        print 'Starting servers...'

        self.tts_server = actionlib.SimpleActionServer(
            c.SOUND_TTS,
            ros_homebot.msg.TTSAction,
            self.say,
            False)
        self.tts_server.start()

        print 'Ready!'

    def cancel_all(self, exclude=None):
        print 'Cancelling all...'
        with self._lock:
            if exclude != c.SOUND_TTS:
                client = actionlib.SimpleActionClient(c.SOUND_TTS, ros_homebot.msg.TTSAction)
                client.wait_for_server()
                client.cancel_all_goals()
        print 'All cancelled.'

    def shutup(self, goal_or_srv):
        self.cancel_all()

    def say(self, goal_or_srv):
        """
        Speaks the given text.
        """
        is_goal = not type(goal_or_srv).__name__.endswith('Request')
        print 'received request to say:', goal_or_srv.text
        self.cancel_all(exclude=c.SOUND_TTS)
        success = True
        try:

#             voice = goal_or_srv.voice.strip() or VOICE1
            voice = VOICE1
#             assert voice in VOICES

#             speed = goal_or_srv.speed #TODO

#             volume = goal_or_srv.volume or 100
#             assert 0 <= volume <= 100
#             if self._last_volume is None or self._last_volume != volume:
#                 os.system('amixer cset numid=1 {percent}%'.format(percent=volume))
#                 self._last_volume = volume

            lines = goal_or_srv.text.strip().split('\n')
            for line in lines:
#                 os.system('nice -n -19 espeak -v%s "%s"' % (voice, line))
                os.system('sudo nice -n -19 espeak -v%s "%s" --stdout | aplay' % (voice, line))

        except Exception as e:
            success = False
            traceback.print_exc(file=sys.stderr)
        finally:
            if is_goal:
                result = ros_homebot.msg.TTSResult()
                if success:
                    self.tts_server.set_succeeded(result)
                else:
                    self.tts_server.set_aborted(result)
        return ros_homebot_msgs.srv.TTSResponse()

    def shutdown(self):
        print 'Shutting down...'
        self.running = False
        #self.cancel_all() # hangs indefinitely?
        print 'Done.'

if __name__ == '__main__':
    rospy.init_node('sound')
    server = SoundServer(rospy.get_name())
    rospy.spin()
