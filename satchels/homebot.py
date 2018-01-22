import os
import time

from burlap.constants import *
from burlap import ServiceSatchel
from burlap.decorators import task, runs_once
from burlap.trackers import FilesystemTracker, SettingsTracker

class HomebotSatchel(ServiceSatchel):

    name = 'homebot'

    def set_defaults(self):

        self.env.project_dir = '/usr/local/homebot'

        #self.env.log_paths = [
            #'/var/log/homebot-head.log',
            #'/var/log/homebot-torso.log',
        #]

        self.env.user = 'ubuntu'

        self.env.group = 'ubuntu'

        self.env.daemon_name = 'homebot'

        self.env.service_commands = {
            START:{
                UBUNTU: 'service supervisor start',
                #UBUNTU: 'service %s start' % self.env.daemon_name,
            },
            STOP:{
            UBUNTU: 'service supervisor stop',
                #UBUNTU: 'service %s stop' % self.env.daemon_name,
            },
            DISABLE:{
                #UBUNTU: 'chkconfig %s off' % self.env.daemon_name,
            },
            ENABLE:{
                #UBUNTU: 'chkconfig %s on' % self.env.daemon_name,
            },
            RESTART:{
            UBUNTU: 'service supervisor restart',
                #UBUNTU: 'service %s restart' % self.env.daemon_name,
            },
            STATUS:{
            UBUNTU: 'service supervisor status',
                #UBUNTU: 'service %s status' % self.env.daemon_name,
            },
        }

        self.env.upstart_setup = '/usr/local/homebot/src/ros/setup.bash'

        self.env.upstart_launch = 'ros_homebot/launch/all.launch'

        self.env.upstart_name = self.name

        self.env.upstart_user = self.env.user

        self.env.auto_upload_firmware = False

    def get_trackers(self):
        parts = [

            SettingsTracker(
                satchel=self,
                names='project_dir',
                action=self.init_path),

            SettingsTracker(
                satchel=self,
                names='project_dir user group',
                action=self.init_project),

            #SettingsTracker(
                #satchel=self,
                #names='log_paths user group',
                #action=self.init_log),

            SettingsTracker(
                satchel=self,
                names='project_dir user group',
                action=self.init_virtualenv),

            FilesystemTracker(
                base_dir='roles', extensions='apt-requirements.txt pip-requirements.txt',
                action=self.update_settings),

            FilesystemTracker(
                base_dir='src/ros/src', extensions='*.py *.launch *.action *.yaml',
                action=self.deploy_code),

            FilesystemTracker(
                base_dir='roles/prod/templates/supervisor', extensions='homebot.conf.template',
                action=self.install_upstart),

            FilesystemTracker(
                base_dir='src/ros/src/ros_homebot/src/firmware/head2', extensions='*.ino *.h *.cpp',
                action=self.make_firmware_head2),

            FilesystemTracker(
                base_dir='src/ros/src/ros_homebot/src/firmware/torso2', extensions='*.ino *.h *.cpp',
                action=self.make_firmware_torso2),

            FilesystemTracker(
                base_dir='roles', extensions='apt-requirements.txt pip-requirements.txt',
                action=self.pip_install),

            FilesystemTracker(
                base_dir='src/ros/src', extensions='*.py *.launch *.action *.yaml',
                action=self.delete_logs),
        ]

        if self.env.auto_upload_firmware:

            parts.extend([

                FilesystemTracker(
                    base_dir='src/ros/src/ros_homebot/src/firmware/head2', extensions='*.ino *.h *.cpp',
                    action=self.upload_firmware_head2),

                FilesystemTracker(
                    base_dir='src/ros/src/ros_homebot/src/firmware/torso2', extensions='*.ino *.h *.cpp',
                    action=self.upload_firmware_torso2),

            ])

        #parts.extend([
            #SettingsTracker(
                #satchel=self,
                #names='upstart_setup upstart_launch upstart_name upstart_user',
                #action=self.install_upstart),

            #TODO:Rebuild messages.

            #TODO:Rebuild C/C++ ROS nodes.
            #r.run('cd {project_dir}/src/ros; . ./setup.bash; catkin_make')
        #])

        return parts

    @task
    def make_firmware_head2(self):
        """
        Cross-compiles the head Arduino firmware.
        """
        r = self.local_renderer
        # CS 2017.1.5 Hoped to cross-compile and upload compiled firmware, because dev computer is much faster than the Pi,
        # but the cross-compiled firmware is not compatible when uploaded.
        #r.local('cd {project_dir}/src/ros/src/ros_homebot/src/firmware/head2; make')
        r.run('cd {project_dir}/src/ros/src/ros_homebot/src/firmware/head2; make clean; make; make upload')

    @task
    def make_firmware_torso2(self):
        """
        Cross-compiles the torso Arduino firmware.
        """
        r = self.local_renderer
        #r.local('cd {project_dir}/src/ros/src/ros_homebot/src/firmware/torso2; make')
        r.run('cd {project_dir}/src/ros/src/ros_homebot/src/firmware/torso2; make clean; make; make upload')

    @task
    def upload_firmware_head2(self):
        """
        Uploads the previously cross-compiled firmware to the head Arduino.
        """
        r = self.local_renderer
        r.run('cd {project_dir}/src/ros/src/ros_homebot/src/firmware/head2; make upload')

    @task
    def upload_firmware_torso2(self):
        """
        Uploads the previously cross-compiled firmware to the head Arduino.
        """
        r = self.local_renderer
        r.run('cd {project_dir}/src/ros/src/ros_homebot/src/firmware/torso2; make upload')

    @task
    def reboot(self, *args, **kwargs):
        return super(HomebotSatchel, self).reboot(*args, **kwargs)

    @task
    def rebuild_messages(self):
        r = self.local_renderer
        r.run('cd /usr/local/homebot/src/ros; . ./setup.bash; time catkin_make '
            '--pkg ros_homebot_msgs')

    @task
    def install_upstart(self, force=0):
        """
        Installs the script to automatically start the ROS core at boot time.

        Should be run like:

            fab prod homebot.install_upstart
        """
        force = int(force)
        r = self.local_renderer

        #CS 2018-1-2 Remove due to instability and unreliability. Obscures log files. Generally unusable. Replaced with supervisor.
        #if force or not r.file_exists('/etc/init/homebot.conf'):
            #r.sudo('source {upstart_setup}; rosrun robot_upstart install '
                #'--setup {upstart_setup} --job {upstart_name} '
                #'--rosdistro kinetic'
                #'--user {upstart_user} {upstart_launch}')
            #r.sudo('systemctl daemon-reload && systemctl start {upstart_name}')
            #r.reboot(wait=300, timeout=60)
        #http://docs.ros.org/jade/api/robot_upstart/html/

        r.install_config(local_path='supervisor/homebot.conf.template', remote_path='/etc/supervisor/conf.d/homebot.conf')

    @task
    def uninstall_upstart(self):
        r = self.local_renderer
        #if r.file_exists('/etc/init/homebot.conf'):
        #r.sudo('source {upstart_setup}; rosrun robot_upstart uninstall {upstart_name}')
        r.sudo('rm /etc/supervisor/conf.d/homebot.conf')

    @task
    def catkin_make(self, pkg=None):
        r = self.local_renderer
        self.stop()
        self.sleep(3)
        if pkg:
            r.env.pkg = pkg
            r.run('cd {project_dir}/src/ros; . ./setup.bash; time catkin_make --pkg {pkg}')
        else:
            r.run('cd {project_dir}/src/ros; . ./setup.bash; time catkin_make')
        self.start()

    @task
    def delete_logs(self):
        r = self.local_renderer
        #r.sudo('rm -Rf /home/{user}/.ros/log/*')
        prompts = {'(y/n)?': 'y'}
        with self.settings(prompts=prompts):
            r.run('cd {project_dir}/src/ros; . ./setup.bash; rosclean purge')

    #DEPRECATED
    @task
    def init_teleop(self):
        r = self.local_renderer
        if not r.file_exists('/usr/local/homebot/src/ros/src/ros_homebot_teleop/data/db.sqlite3'):
            r.run('. {project_dir}/src/ros/setup.bash; '
                'cd /usr/local/homebot/src/ros/src/ros_homebot_teleop/src; '
                './manage.py migrate --run-syncdb')
            r.run('. {project_dir}/src/ros/setup.bash; '
                'cd /usr/local/homebot/src/ros/src/ros_homebot_teleop/src; '
                './manage.py loaddata homebot_dashboard/fixtures/users.json')

    @task
    def build_raspicam(self):
        assert os.path.isdir('src/overlay/src/raspicam_node')
        r = self.local_renderer
        r.local('rsync --recursive --verbose --perms --times --links --compress --copy-links '
            '--exclude=.build --exclude=build --exclude=devel '
            '--exclude=.build_ano '
            '--exclude=db.sqlite3 '
            '--exclude=.env '
            '--exclude=.git '
            '--exclude=setup_local.bash '
            '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" '
            'src/overlay/src/raspicam_node {user}@{host_string}:{project_dir}/src/overlay/src')
        r.run('cd {project_dir}/src/overlay; . {project_dir}/src/ros/setup.bash; '
            'time catkin_make --pkg raspicam')

    @task
    def update_settings(self):
        r = self.local_renderer
        r.local('mkdir -p src/settings/{ROLE}')
        r.local('cp roles/all/apt-requirements.txt src/settings/{ROLE}')
        r.local('cp roles/all/pip-requirements.txt src/settings/{ROLE}')

    @task
    def upload_rosserial(self):
        # Uploads our custom rosserial branch for testing.
        r = self.local_renderer
        r.sudo('chown -R ubuntu:ubuntu /opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_python')
        r.sudo('mkdir -p /opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_arduino; ' \
            'chown -R ubuntu:ubuntu /opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_arduino')
        r.sudo('mkdir -p /opt/ros/kinetic/lib/rosserial_arduino; ' \
            'chown -R ubuntu:ubuntu /opt/ros/kinetic/lib/rosserial_arduino')

        # rosserial_python
        r.local('rsync --recursive --verbose --perms --times --links --compress '
            '--rsh "ssh -t -o StrictHostKeyChecking=no -i roles/prod/rae-ubuntu.pem" '
            '../rosserial/rosserial_python/src/rosserial_python/*  {user}@{host_string}:/opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_python/')

        # rosserial_arduino
        r.local('rsync --recursive --verbose --perms --times --links --compress '
            '--rsh "ssh -t -o StrictHostKeyChecking=no -i roles/prod/rae-ubuntu.pem" '
            '../rosserial/rosserial_arduino/src/rosserial_arduino/__init__.py '
            '{user}@{host_string}:/opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_arduino/__init__.py')
        r.local('rsync --recursive --verbose --perms --times --links --compress '
            '--rsh "ssh -t -o StrictHostKeyChecking=no -i roles/prod/rae-ubuntu.pem" '
            '../rosserial/rosserial_arduino/src/rosserial_arduino/SerialClient.py '
            '{user}@{host_string}:/opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_arduino/SerialClient.py')
        r.local('rsync --recursive --verbose --perms --times --links --compress '
            '--rsh "ssh -t -o StrictHostKeyChecking=no -i roles/prod/rae-ubuntu.pem" '
            '../rosserial/rosserial_arduino/nodes/serial_node.py  {user}@{host_string}:/opt/ros/kinetic/lib/rosserial_arduino/serial_node.py')

        r.sudo('chown -R root:root /opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_python')
        r.sudo('chown -R root:root /opt/ros/kinetic/lib/python2.7/dist-packages/rosserial_arduino')
        r.sudo('chown -R root:root /opt/ros/kinetic/lib/rosserial_arduino')

    @task
    def deploy_code(self):
        r = self.local_renderer

        if not r.genv.key_filename:
            r.genv.key_filename = self.genv.host_original_key_filename

        r.sudo('mkdir -p {project_dir}')
        r.sudo('chown {user}:{user} {project_dir}')

        #archive=-rlptgoD (we don't want g=groups or o=owner or D=devices because remote system
        # has different permissions and hardware)
        r.local('rsync --recursive --verbose --perms --times --links --compress --copy-links '
            '--log-file=./deloy_code.log '
            '--exclude=.build --exclude=build --exclude=devel '
            '--exclude=overlay '
            '--exclude=.build_ano '
            '--exclude=build-uno_pro '
            '--exclude=db.sqlite3 '
            '--exclude=bags '
            '--exclude=.env '
            '--exclude=setup_local.bash '
            '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" '
            'src {user}@{host_string}:{project_dir}')
            #'--exclude=build-* '

        #TODO:remove once lib stable
#         r.local('rsync --recursive --verbose --perms --times --links --compress --copy-links '
#             '--exclude=.build --exclude=build --exclude=devel '
#             '--exclude=overlay '
#             '--exclude=.build_ano '
#             '--exclude=db.sqlite3 '
#             '--exclude=.env '
#             '--exclude=setup_local.bash '
#             '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" '
#             '/home/chris/git/i2cdevlib {user}@{host_string}:/usr/share/arduino/libraries/')

    @task
    def view_log(self):
        r = self.local_renderer
        r.env.log_path = '/var/log/supervisor-homebot.log'
        r.run('tail -f {log_path}')

    @task
    def init_path(self):
        """
        Add our custom bin folder to our path.
        """
        r = self.local_renderer
        text = 'PATH={project_dir}/src/bin:$PATH'.format(**r.env)
        if not r.file_contains(filename='~/.bash_aliases', text=text):
            r.append(text=text, filename='~/.bash_aliases')

    @task
    def init_project(self):
        """
        Initializes our project's base directory.
        """
        r = self.local_renderer
        r.sudo('mkdir -p {project_dir}; chown {user}:{group} {project_dir}')

    @task
    def init_virtualenv(self):
        """
        Initializes the Python virtual environment.
        """
        r = self.local_renderer
        if not r.file_exists(r.env.project_dir+'/.env'):
            #r.sudo('mkdir -p {project_dir}; chown {user}:{group} {project_dir}')
            r.sudo('cd {project_dir}; virtualenv --system-site-packages .env')
            r.sudo('chown -R {user}:{group} {project_dir}')
            r.sudo('cd {project_dir}; . .env/bin/activate; pip install -U pip')

    @task
    def pip_install(self):
        """
        Installs all required Python packages into our virtual environment.
        """
        r = self.local_renderer
        if self.genv.is_local:
            r.env.project_dir = '.'
        r.run('{project_dir}/.env/bin/pip install -U pip')
        r.run('{project_dir}/.env/bin/pip install --only-binary numpy,matplotlib,scipy -r {project_dir}/src/settings/{ROLE}/pip-requirements.txt')

    #@task
    #def init_log(self):
        #"""
        #Initializes our custom log files.
        #"""
        #r = self.local_renderer
        #for log_path in self.lenv.log_paths:
            #r.env.log_path = log_path
            #if not r.file_exists(log_path):
                #r.sudo('touch {log_path}; chown {user}:{group} {log_path}')

    @task
    @runs_once
    def firmware_shell(self, *args, **kwargs):
        debug = self.get_satchel('debug')
        debug.shell(shell_interactive_cmd_str='cd %s/src/ros/src/ros_homebot/src/firmware; /bin/bash -i;' % self.env.project_dir, *args, **kwargs)

    @task
    def view_camera(self):
        """
        Views the camera on the remote robot from the local ROS installation.

        To be run like:

            fab prod homebot.view_camera

        Assumes raspicam_compressed_320.launch is running on the robot.
        """
        r = self.local_renderer
        #self.genv.shell = '/bin/bash'
        r.local(
            'bash -c "cd src/ros; source ./setup.bash; export ROS_MASTER_URI=http://{host_string}:11311; '
            'rosrun image_view image_view image:=/raspicam/image _image_transport:=compressed"')

    @task
    def view_diagnostics(self):
        """
        Views diagnostics on the remote robot from the local ROS installation.

        To be run like:

            fab prod homebot.view_diagnostics

        """
        r = self.local_renderer
        r.local(
            'bash -c "cd src/ros; source ./setup.bash; export ROS_MASTER_URI=http://{host_string}:11311; '
            'rosrun rqt_robot_monitor rqt_robot_monitor"')

    @task
    def refesh_dns(self):
        """
        Clears DNS. Useful for when the robot has rebooted and it's local network name no longer resolves to the old IP.

        To be run like:

            fab local homebot.refresh_dns

        """
        r = self.local_renderer
        r.sudo('/etc/init.d/dns-clean restart; /etc/init.d/networking force-reload')

    @task
    def test_sound(self):
        r = self.local_renderer
        r.sudo('aplay /usr/share/sounds/alsa/Front_Center.wav')

    @task(precursors=['packager', 'user', 'ros', 'rpi', 'ntp', 'ntpclient'])
    def configure(self):
        self.stop()
        super(HomebotSatchel, self).configure() # runs tasks triggered by trackers
        self.start()
        time.sleep(20) # wait for rosmaster to start
        r = self.local_renderer
        r.run('cd {project_dir}/src/ros; . ./setup.bash; roswtf')

homebot = HomebotSatchel()
