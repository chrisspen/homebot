"""
Arduino
===================

Tools for configuring software and features specific to systems interacting
with Arduino microcontrollers.
"""

from fabric.api import settings

from burlap import Satchel
from burlap.constants import *
from burlap.decorators import task

class ArduinoSatchel(Satchel):
    """
    Installs the Arduino IDE and dependencies.

    Note, this satchel depends on the ROS satchel, and extends it.
    """

    name = 'arduino'

    def set_defaults(self):
        # Find most recent version at https://www.arduino.cc/en/Main/Software
        #self.env.version = '1.6.1'
        self.env.version = '1.8.3'

        # This is the 1.5.* location.
        #self.env.base_variants_dir = '/usr/share/arduino/hardware/arduino/variants'
        # This is the 1.8.* location.
        self.env.base_variants_dir = '/usr/share/arduino/hardware/arduino/avr/variants'

        # This is the 1.5.* location.
        #self.env.boards_path = '/usr/share/arduino/hardware/arduino/boards.txt'
        # This is the 1.8.* location.
        self.env.boards_path = '/usr/share/arduino/hardware/arduino/avr/boards.txt'

        self.env.sketchbook_path = '/home/{user}/sketchbook'

    @property
    def packager_system_packages(self):
        return {
            UBUNTU: [
                #'arduino',#too old
                #'arduino-core',
                #'arduino-mk',
                'avrdude',
                'avr-libc',
                'gcc-avr',
                'picocom',
                'git',
                'python-setuptools',
                'python-dev',
                'ros-kinetic-rosserial',
                'ros-kinetic-rosserial-arduino',
                'ros-kinetic-rosserial-python',
            ],
        }

    @task
    def install(self):
        r = self.local_renderer

        # Setup the Arduino IDE.
        # http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
        # Do this instead in each Arduino project's ./lib directory.
        #r.run('. {ros_source_path}; '
            #'cd {sketchbook_path}; mkdir libraries; cd libraries; rm -rf ros_lib; '
            #'rosrun rosserial_arduino make_libraries.py .')

        # Clear the old Arduino IDE installed by the obsolete arduino package, which is unfortunately required by the rosserial-arduino package.
        r.sudo('rm -Rf /usr/share/arduino/*')

        # Install Arduino-Makefile
        r.run('cd /tmp; [ -d Arduino-Makefile ] && rm -Rf Arduino-Makefile || true; '
            'git clone https://github.com/sudar/Arduino-Makefile.git; '
            'sudo mkdir -p /usr/share/arduino; sudo cp -R ./Arduino-Makefile/* /usr/share/arduino/')

        # Determine target architecture.
        arch = (r.run('uname -m') or '').strip() or 'x86_64'
        if arch == 'x86_64':
            r.env.fname = 'linux64'
        elif 'arm' in arch:
            r.env.fname = 'linuxarm'
        else:
            raise NotImplementedError('Unknown architecture: %s' % arch)

        # Download and install Arduino IDE.
        r.run('cd /tmp; wget https://www.arduino.cc/download.php?f=/arduino-{version}-{fname}.tar.xz -O arduino.tar.xz; tar -xJf arduino.tar.xz')
        r.sudo('cp -R /tmp/arduino-{version}/* /usr/share/arduino')

        # Install Arduino IDE support files for Arduino Uno*Pro.
        if not r.file_contains(r.env.boards_path, 'Arduino Uno*Pro'):
            r.append(filename=r.env.boards_path, text='''
##############################################################

uno_pro.name=Arduino Uno*Pro

uno_pro.upload.tool=avrdude
uno_pro.upload.protocol=arduino
uno_pro.upload.maximum_size=130048
uno_pro.upload.maximum_data_size=16384
uno_pro.upload.speed=115200

uno_pro.bootloader.tool=avrdude
uno_pro.bootloader.low_fuses=0xFF
uno_pro.bootloader.high_fuses=0xDE
uno_pro.bootloader.extended_fuses=0xFD
uno_pro.bootloader.unlock_bits=0x3F
uno_pro.bootloader.lock_bits=0x0F
uno_pro.bootloader.file=optiboot/optiboot_atmega1284p.hex

uno_pro.build.mcu=atmega1284p
uno_pro.build.f_cpu=16000000L
uno_pro.build.board=AVR_UNO_PRO
uno_pro.build.core=arduino
uno_pro.build.variant=uno_pro

##############################################################
''', use_sudo=True)
        r.sudo('mkdir -p {base_variants_dir}/uno_pro')
        path = self.find_template('arduino/pins_arduino.h')
        r.put(local_path=path, remote_path='{base_variants_dir}/uno_pro/pins_arduino.h', use_sudo=True)

        # Symlink our custom libraries to the sketchbook.
        # Note, our sketchbook path must match USER_LIB_PATH in the Makefile.
        #TODO:remove this when support for multiple custom library paths is supported by Arduino-Makefile
        #lib_paths = [
            #'src/ros/src/ros_homebot/src/firmware/head2/lib',
            #'src/ros/src/ros_homebot/src/firmware/torso2/lib',
        #]
        #for lib_path in lib_paths:
            #for lib_name in os.listdir(lib_path):
                #fq_lib_name = os.path.join(r.genv.project_home, lib_path, lib_name)
                #r.env.fq_lib_name = fq_lib_name
                #r.env.lib_name = lib_name
                #r.run('[ ! -e {sketchbook_path}/libraries/{lib_name} ] && ln -s {fq_lib_name} {sketchbook_path}/libraries/ || true')

    @task
    def uninstall(self):
        r = self.local_renderer
        r.sudo('rm -Rf /usr/share/arduino')
        #r.sudo("sed -i '/^uno_pro/d' {boards_path}")
        #r.sudo('rm -f {base_variants_dir}/uno_pro/pins_arduino.h')

    @task
    def init_sketchbook(self):
        r = self.local_renderer
        r.run('mkdir -p ~/sketchbook/libraries')

    @task(precursors=['packager', 'user', 'ros'])
    def configure(self):
        r = self.local_renderer

        # Install Arduino from repo.
        # Note, we have to install arduino-core (which is an ancient obsolute version of Arduino IDE)
        # because it's required by rosserial-arduino.
        # However, we have to delete everything it installs to /usr/share/arduino,
        # otherwise it break Arduino-Makefile.
        self.install_packages()

        # Allow user to upload code to arduino.
        r.sudo('usermod -a -G dialout $USER')

        # Install Arduino IDE, makefile and other utilities.
        self.install()

        # Fixes error avrdude.conf not found.
        r.sudo('mkdir -p /usr/share/arduino/hardware/tools/avr/etc')
        with settings(warn_only=True):
            r.sudo('[ ! -e /usr/share/arduino/hardware/tools/avr/etc/avrdude.conf ] && '
                'ln -s /usr/share/arduino/hardware/tools/avrdude.conf /usr/share/arduino/hardware/tools/avr/etc/ || true')

arduino = ArduinoSatchel()
