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
    """

    name = 'arduino'
    
    def set_defaults(self):
        self.env.version = '1.6.1'
        self.env.arch = 32
    
    @property
    def packager_system_packages(self):
        return {
            UBUNTU: [
                'arduino',
                'arduino-core',
                'picocom',
                'git',
                'python-setuptools',
                'python-dev',
            ],
            DEBIAN: [
                'arduino',
                'arduino-core',
                'picocom',
                'git',
                'python-setuptools',
                'python-dev',
            ],
        }
        
    @task
    def configure(self):
        r = self.local_renderer
        
        # Install Arduino from repo.
        #r.sudo('apt-get update && apt-get install arduino arduino-core picocom')
        #self.install_packages()
        
        # Fixes error avrdude.conf not found.
        r.sudo('mkdir -p /usr/share/arduino/hardware/tools/avr/etc')
        with settings(warn_only=True):
            r.sudo('ln -s /usr/share/arduino/hardware/tools/avrdude.conf '
                '/usr/share/arduino/hardware/tools/avr/etc/')
        
        # Install Arturo.
        r.sudo('rm -Rf /tmp/Arturo')
        r.sudo('cd /tmp; git clone https://github.com/scottdarch/Arturo.git')
        r.sudo('cd /tmp/Arturo; make install')
        
    configure.deploy_before = ['packager', 'user']

arduino = ArduinoSatchel()
