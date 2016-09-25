import time
import os

from burlap.constants import *
from burlap import ServiceSatchel
from burlap.decorators import task

class HomebotSatchel(ServiceSatchel):

    name = 'homebot'

    def set_defaults(self):
        
        self.env.project_dir = '/usr/local/homebot'
        
        self.env.log_paths = [
            '/var/log/homebot-head.log',
            '/var/log/homebot-torso.log',
        ]
        
        self.env.user = 'ubuntu'
        
        self.env.group = 'ubuntu'
        
        self.env.daemon_name = 'homebot'
        
        self.env.autostart_script = 'homebot_autostart_screens'
        self.env.start_script = 'homebot_start'
        self.env.restart_script = 'homebot_restart'
        self.env.stop_script = 'homebot_stop'
        self.env.status_script = 'homebot_status'
        self.env.cron_script = 'homebot_crontab'
        self.env.cron_dir = '/etc/cron.d'
        
        self.env.service_commands = {
            START:{
                UBUNTU: 'service %s start' % self.env.daemon_name,
                RASPBIAN: 'homebot_start.sh',
            },
            STOP:{
                UBUNTU: 'service %s stop' % self.env.daemon_name,
                RASPBIAN: 'homebot_stop.sh',
            },
            DISABLE:{
                UBUNTU: 'chkconfig %s off' % self.env.daemon_name,
                RASPBIAN: self.uninstall_autostart_cron,
            },
            ENABLE:{
                UBUNTU: 'chkconfig %s on' % self.env.daemon_name,
                RASPBIAN: self.install_autostart_cron,
            },
            RESTART:{
                UBUNTU: 'service %s restart' % self.env.daemon_name,
                RASPBIAN: 'homebot_restart.sh',
            },
            STATUS:{
                UBUNTU: 'service %s status' % self.env.daemon_name,
                RASPBIAN: 'homebot_status.sh',
            },
        }

        self.env.upstart_setup = '/usr/local/homebot/src/ros/setup.bash'
        
        self.env.upstart_launch = 'ros_homebot/launch/all.launch'
        
        self.env.upstart_name = self.name
        
        self.env.upstart_user = self.env.user
        
        self.env.pip_requirements = [
            #'Fabric==1.10.2',
            #'PyYAML==3.11',
            #'burlap',
            
            #DEPRECATED: Installed by ROS?
            #'wiringpi2==1.1.1',
            
            'Pillow==2.3.0',
            'picamera==1.10',
            'numpy==1.8.2',
            
            'pint==0.7.2',
            
            # head_arduino_tester
            'pyserial==3.0.1',
            'urwid==1.3.1',
            
            # needed by several nodes, including lrf
            'scipy==0.18.0',
            
            # Required by ros_homebot_lrf.
            #laser_range_finder
            #-e git://github.com/chrisspen/laser-range-finder.git#egg=laser_range_finder
            
            'humanize==0.5.1',
            
            # Needed for ROS message generation.
            'empy==3.3.2',
            
            'defusedxml',
            'rospkg',
            
            # Teleop
            'Django==1.9.2',
            '-e git://github.com/chrisspen/gevent-socketio.git#egg=gevent-socketio',
        ]
    
    @property
    def packager_system_packages(self):
        pkgs = [
        
            # Python
            'python-dev',
            'python-pip',
            'python-virtualenv',
            
            'dphys-swapfile',
            #'linux-firmware',#not found on raspbian?
            'openssh-server',
            'network-manager',
            'wireless-tools',
            'rsync',
            
            # Debugging helpers.
            'htop',
            'curl',
            'screen',
            'git',
            
            # Needed by numpy/scipy
            'libatlas-base-dev',
            'gcc',
            'gfortran',
            'g++',
            'libblas-dev',
            'liblapack-dev',
            'gfortran',
            
            # Speech output.
            'espeak',
            
            'alsa-utils',
            'mpg321',
            'lame',
            
            #ros-kinetic-image-view
            ##ros-kinetic-mjpeg-server
            ##ros-kinetic-web-video-server
            ##ros-kinetic-robot-upstart #TODO:fix
            #ros-kinetic-xacro
            #ros-kinetic-robot-state-publisher
            
#             'ros-%s-image-view' % self.genv.ros_version_name,
#             'ros-%s-rqt-robot-monitor' % self.genv.ros_version_name,

        ]
        return {
            UBUNTU: pkgs,
            DEBIAN: pkgs,
            RASPBIAN: pkgs + ['upstart'],#TODO:remove once robot_upstart supports systemd
        }
    
    @task
    def reboot(self, *args, **kwargs):
        return super(HomebotSatchel, self).reboot(*args, **kwargs)
    
    @task
    def rebuild_messages(self):
        r = self.local_renderer
        r.run('cd /usr/local/homebot/src/ros; . ./setup.bash; time catkin_make --pkg ros_homebot_msgs')
    
    #2016.8.14 CKS Removed due to package unsupported on Kinetic.
#     @task
#     def install_upstart(self, force=0):
#         """
#         Installs the upstart script for automatically starting your application.
#         
#         http://docs.ros.org/api/robot_upstart/html/
#         """
#         force = int(force)
#         print('force:',force)
#         r = self.local_renderer
#         #r.sudo('apt-get install ros-indigo-robot-upstart')
#         if force or not r.file_exists('/etc/init/homebot.conf'):
#             r.sudo('source /usr/local/homebot/src/ros/setup.bash; rosrun robot_upstart install --setup {upstart_setup} --job {upstart_name} --user {upstart_user} {upstart_launch}')
#             
#             r.reboot(wait=300, timeout=60)
    
#     @task
#     def uninstall_upstart(self):
#         r = self.local_renderer
#         if r.file_exists('/etc/init/homebot.conf'):
#             r.sudo('rosrun robot_upstart uninstall {upstart_name}')

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
    def install_autostart(self):
        """
        Installs cron-based scripts to start ROS when the system boots.
        """
        r = self.local_renderer
        #r.env.log_dir = '/home/{user}/homebot_autostart/logs'.format(**r.genv)
        r.env.log_dir = '/var/log'
        r.env.source_path = '/usr/local/homebot/src/ros/setup.bash'
        r.env.script_dir = self.env.script_dir = '/usr/local/bin'
        r.run('mkdir -p {log_dir}')
        
        r.sudo('touch {log_dir}/{autostart_script}.log')
        r.sudo('chown {user}:{user} {log_dir}/{autostart_script}.log')
        r.sudo('touch {log_dir}/{start_script}.log')
        r.sudo('chown {user}:{user} {log_dir}/{start_script}.log')
        
        r.pc('Installing scripts.')
        r.install_script(
            local_path='homebot/{autostart_script}.sh',
            remote_path='{script_dir}/{autostart_script}.sh',
            extra=r.env)
        r.install_script(
            local_path='homebot/{start_script}.sh',
            remote_path='{script_dir}/{start_script}.sh',
            extra=r.env)
        r.install_script(
            local_path='homebot/{stop_script}.sh',
            remote_path='{script_dir}/{stop_script}.sh',
            extra=r.env)
        r.install_script(
            local_path='homebot/{status_script}.sh',
            remote_path='{script_dir}/{status_script}.sh',
            extra=r.env)
        r.install_script(
            local_path='homebot/{restart_script}.sh',
            remote_path='{script_dir}/{restart_script}.sh',
            extra=r.env)
        
        self.install_autostart_cron()
    
    @task
    def install_autostart_cron(self):
        r = self.local_renderer
        r.pc('Installing crontab.')
        r.install_script(
            local_path='homebot/{cron_script}',
            remote_path='{cron_dir}/{cron_script}',
            extra=r.env)
        r.sudo('chown root:root /etc/cron.d/homebot_crontab')
        r.sudo('chmod 600 /etc/cron.d/homebot_crontab')
        r.sudo('service cron restart')
        
    @task
    def uninstall_autostart_cron(self):
        r = self.local_renderer
        r.sudo('rm -f {cron_dir}/{cron_script}')
    
    @task
    def delete_logs(self):
        r = self.local_renderer
        r.sudo('rm -Rf /home/pi/.ros/log/*')
    
    @task
    def install_bcm2835(self):
        r = self.local_renderer
        r.run('cd /tmp; wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.50.tar.gz; tar zxvf bcm2835-1.50.tar.gz')
        r.run('cd bcm2835-1.50; ./configure')
        r.run('cd bcm2835-1.50; time make')
        r.sudo('cd bcm2835-1.50; make check')
        r.sudo('cd bcm2835-1.50; make install')
    
    @task
    def install_i2cdevlib(self):
        r = self.local_renderer
        #TODO:switch back to https://github.com/jrowberg/i2cdevlib/ when pull request 258 accepted
        #https://github.com/jrowberg/i2cdevlib/pull/258
        r.sudo('cd /usr/share/arduino/libraries; sudo git clone https://github.com/chrisspen/i2cdevlib.git')
    
    @task
    def init_teleop(self):
        r = self.local_renderer
        if not r.file_exists('/usr/local/homebot/src/ros/src/ros_homebot_teleop/data/db.sqlite3'):
            r.run('/usr/local/homebot/src/ros/src/ros_homebot_teleop/src; ./manage.py migrate --run-syncdb')
            r.run('/usr/local/homebot/src/ros/src/ros_homebot_teleop/src; ./manage.py loaddata homebot_dashboard/fixtures/users.json')
    
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
            '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" src/overlay/src/raspicam_node {user}@{host_string}:{project_dir}/src/overlay/src')
        r.run('cd {project_dir}/src/overlay; . {project_dir}/src/ros/setup.bash; time catkin_make --pkg raspicam')
    
    @task
    def deploy_code(self):
        
        r = self.local_renderer
        
        if not r.genv.key_filename:
            r.genv.key_filename = self.genv.host_original_key_filename
        
        #archive=-rlptgoD (we don't want g=groups or o=owner or D=devices because remote system has different permissions and hardware)
        r.local('rsync --recursive --verbose --perms --times --links --compress --copy-links '
            '--exclude=.build --exclude=build --exclude=devel '
            '--exclude=overlay '
            '--exclude=.build_ano '
            '--exclude=db.sqlite3 '
            '--exclude=bags '
            '--exclude=.env '
            '--exclude=setup_local.bash '
            '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" src {user}@{host_string}:{project_dir}')
        
        #TODO:remove once lib stable
        r.local('rsync --recursive --verbose --perms --times --links --compress --copy-links '
            '--exclude=.build --exclude=build --exclude=devel '
            '--exclude=overlay '
            '--exclude=.build_ano '
            '--exclude=db.sqlite3 '
            '--exclude=.env '
            '--exclude=setup_local.bash '
            '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" /home/chris/git/i2cdevlib {user}@{host_string}:/usr/share/arduino/libraries/')
    
    @task
    def view_head_log(self):
        r = self.local_renderer
        r.env.log_path = '/var/log/homebot-head.log'
        r.run('tail -f {log_path}')
    
    @task
    def view_torso_log(self):
        r = self.local_renderer
        r.env.log_path = '/var/log/homebot-torso.log'
        r.run('tail -f {log_path}')
    
    @task
    def init_virtualenv(self):
        r = self.local_renderer
        if not r.file_exists(r.env.project_dir+'/.env'):
            #r.sudo('mkdir -p {project_dir}')
            r.sudo('cd {project_dir}; virtualenv --system-site-packages .env')
            r.sudo('chown -R {user}:{group} {project_dir}')
            r.sudo('cd {project_dir}; . .env/bin/activate; pip install -U pip')
    
    @task
    def pip_install(self):
        r = self.local_renderer
        for package in r.env.pip_requirements:
            r.env.package = package
            r.run('{project_dir}/.env/bin/pip install {package}')
    
    @task
    def configure(self):
        
        r = self.local_renderer
        
        lm = self.last_manifest
        lm_pip_requirements = lm.get('pip_requirements', [])
        
        self.install_bcm2835()
        
        self.install_i2cdevlib()
        
        # Add our custom bin folder to our path.
        text = 'PATH={project_dir}/src/bin:$PATH'.format(**r.env)
        if not r.file_contains(filename='~/.bash_aliases', text=text):
            r.append(text=text, filename='~/.bash_aliases')
        
        # Initialize our log file.
        for log_path in self.lenv.log_paths:
            r.env.log_path = log_path
            if not r.file_exists(log_path):
                r.sudo('touch {log_path}; chown {user}:{group} {log_path}')
        
        # Initialize project home.
        r.sudo('mkdir -p {project_dir}; chown {user}:{group} {project_dir}')
        
        # Initialize Python virtualenv.
        self.init_virtualenv()
        
        # Install all PIP package.
        if lm_pip_requirements != self.env.pip_requirements:
            self.pip_install()
        
        # Push up all code changes.
        self.deploy_code()
        
        r.run('cd {project_dir}/src/ros; . ./setup.bash; catkin_make')
        
        self.init_teleop()
        
        #self.install_upstart()#Removed since package does not support kinetic
        self.install_autostart()
        
    configure.is_deployer = True
    configure.deploy_before = ['packager', 'user', 'ros', 'rpi', 'ntp', 'ntpclient']

homebot = HomebotSatchel()
