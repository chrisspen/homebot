#!/usr/bin/env python
"""
An improved version of the serial_node.py from rosserial_python, implementing Arduino reset.

https://github.com/ros-drivers/rosserial/blob/jade-devel/rosserial_python/src/rosserial_python/SerialClient.py
"""

import time
from time import sleep
import multiprocessing
import sys
import struct

#import std_srvs.srv
from std_srvs.srv import Empty, EmptyResponse
import rospy
from rosserial_python import SerialClient as _SerialClient, RosSerialServer
import diagnostic_msgs.msg

from serial import Serial
from serial import SerialException

class SerialClient(_SerialClient):

    def __init__(self, *args, **kwargs):
        kwargs.pop('fix_pyserial_for_test', None)
        _SerialClient.__init__(self, *args, **kwargs)
        rospy.Service('~hard_reset', Empty, self.hard_reset)

    def tryRead(self, length):
        with self.mutex:
            try:
                read_start = time.time()
                read_current = read_start
                bytes_remaining = length
                result = bytearray()
                while bytes_remaining != 0 and read_current - read_start < self.timeout:
                    received = self.port.read(bytes_remaining)
                    if len(received) != 0:
                        result.extend(received)
                        bytes_remaining -= len(received)
                    read_current = time.time()

                if bytes_remaining != 0:
                    rospy.logwarn("Serial Port read returned short (expected %d bytes, received %d instead)."
                                  % (length, length - bytes_remaining))
                    raise IOError()

                return bytes(result)
            except Exception as e:
                rospy.logwarn("Serial Port read failure: %s", e)
                raise IOError()

    def run(self):
        """ Forward recieved messages to appropriate publisher. """
        data = ''
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.lastsync).to_sec() > (self.timeout * 3):
                if self.synced:
                    rospy.logerr("Lost sync with device, restarting...")
                else:
                    rospy.logerr("Unable to sync with device; possible link problem or link software version "
                        "mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = rospy.Time.now()
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "no sync with device")
                self.requestTopics()
                self.lastsync = rospy.Time.now()

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
            try:
                with self.mutex:
                    if self.port.inWaiting() < 1:
                        time.sleep(0.001)
                        continue

                flag = [0, 0]
                flag[0] = self.tryRead(1)
                if (flag[0] != '\xff'):
                    continue

                flag[1] = self.tryRead(1)
                if flag[1] != self.protocol_ver:
                    self.sendDiagnostics(
                        diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                        "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client")
                    rospy.logerr("Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client")
                    protocol_ver_msgs = {'\xff': 'Rev 0 (rosserial 0.4 and earlier)', '\xfe': 'Rev 1 (rosserial 0.5+)', '\xfd': 'Some future rosserial version'}
                    if (flag[1] in protocol_ver_msgs):
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    rospy.loginfo("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                msg_len_bytes = self.tryRead(2)
                msg_length, = struct.unpack("<h", msg_len_bytes)

                msg_len_chk = self.tryRead(1)
                msg_len_checksum = sum(map(ord, msg_len_bytes)) + ord(msg_len_chk)

                if msg_len_checksum % 256 != 255:
                    rospy.loginfo("wrong checksum for msg length, length %d" %(msg_length))
                    rospy.loginfo("chk is %d" % ord(msg_len_chk))
                    continue

                # topic id (2 bytes)
                topic_id_header = self.tryRead(2)
                topic_id, = struct.unpack("<h", topic_id_header)

                try:
                    msg = self.tryRead(msg_length)
                except IOError:
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Packet Failed : Failed to read msg data")
                    rospy.loginfo("Packet Failed :  Failed to read msg data")
                    rospy.loginfo("expected msg length is %d", msg_length)
                    raise

                # checksum for topic id and msg
                chk = self.tryRead(1)
                checksum = sum(map(ord, topic_id_header)) + sum(map(ord, msg)) + ord(chk)

                if checksum % 256 == 255:
                    self.synced = True
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        rospy.logerr("Tried to publish before configured, topic id %d" % topic_id)
                        self.requestTopics()
                    rospy.sleep(0.001)
                else:
                    rospy.loginfo("wrong checksum for topic id and msg")

            except IOError:
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                self.requestTopics()

    def hard_reset(self, *args, **kwargs):
        """
        Forces the Arduino to perform a hard reset, as though the reset button was pressed.
        """
        print('hard_reset:', args, kwargs)
        with self.mutex:
            print('Closing serial port...')
            self.port.close()
            print('Resetting Arduino on port %s...' % self.port.portstr)
            with Serial(self.port.portstr) as arduino:
                arduino.setDTR(False)
                sleep(3)
                arduino.flushInput()
                arduino.setDTR(True)
                sleep(5)
            print('Arduino reset. Reopening serial port...')
            self.port.open()
            print('Done.')
        return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node("serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud', '57600'))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

    # TODO: should these really be global?
    tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))
    fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2:
        port_name = sys.argv[1]
    if len(sys.argv) == 3:
        tcp_portnum = int(sys.argv[2])

    if port_name == "tcp":
        server = RosSerialServer(tcp_portnum, fork_server)
        rospy.loginfo("Waiting for socket connections on port %d" % tcp_portnum)
        try:
            server.listen()
        except KeyboardInterrupt:
            rospy.loginfo("got keyboard interrupt")
        finally:
            rospy.loginfo("Shutting down")
            for process in multiprocessing.active_children():
                rospy.loginfo("Shutting down process %r", process)
                process.terminate()
                process.join()
            rospy.loginfo("All done")

    else:          # Use serial port
        while not rospy.is_shutdown():
            rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
            try:
                client = SerialClient(port_name, baud, fix_pyserial_for_test=fix_pyserial_for_test)
                client.run()
            except KeyboardInterrupt:
                break
            except SerialException:
                sleep(1.0)
                continue
            except OSError:
                sleep(1.0)
                continue
