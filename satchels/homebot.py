import os

from burlap.constants import *
from burlap import ServiceSatchel
from burlap.decorators import task
from burlap.trackers import FilesystemTracker, SettingsTracker

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

        self.env.service_commands = {
            START:{
                UBUNTU: 'service %s start' % self.env.daemon_name,
            },
            STOP:{
                UBUNTU: 'service %s stop' % self.env.daemon_name,
            },
            DISABLE:{
                UBUNTU: 'chkconfig %s off' % self.env.daemon_name,
            },
            ENABLE:{
                UBUNTU: 'chkconfig %s on' % self.env.daemon_name,
            },
            RESTART:{
                UBUNTU: 'service %s restart' % self.env.daemon_name,
            },
            STATUS:{
                UBUNTU: 'service %s status' % self.env.daemon_name,
            },
        }

        self.env.upstart_setup = '/usr/local/homebot/src/ros/setup.bash'

        self.env.upstart_launch = 'ros_homebot/launch/all.launch'

        self.env.upstart_name = self.name

        self.env.upstart_user = self.env.user

    def get_trackers(self):
        return [

            SettingsTracker(
                satchel=self,
                names='project_dir',
                action=self.init_path),

            SettingsTracker(
                satchel=self,
                names='project_dir user group',
                action=self.init_project),

            SettingsTracker(
                satchel=self,
                names='log_paths user group',
                action=self.init_log),

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
                base_dir='roles', extensions='apt-requirements.txt pip-requirements.txt',
                action=self.pip_install),

            SettingsTracker(
                satchel=self,
                names='upstart_setup upstart_launch upstart_name upstart_user',
                action=self.install_upstart),

            #TODO:Rebuild messages.

            #TODO:Rebuild C/C++ ROS nodes.
            #r.run('cd {project_dir}/src/ros; . ./setup.bash; catkin_make')
        ]

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
        Installs the upstart script for automatically starting your application.

        http://docs.ros.org/api/robot_upstart/html/
        """
        force = int(force)
        r = self.local_renderer
        if force or not r.file_exists('/etc/init/homebot.conf'):
            r.sudo('source {upstart_setup}; rosrun robot_upstart install '
                '--setup {upstart_setup} --job {upstart_name} '
                '--user {upstart_user} {upstart_launch}')
            r.sudo('systemctl daemon-reload && systemctl start {upstart_name}')
            #r.reboot(wait=300, timeout=60)

    @task
    def uninstall_upstart(self):
        r = self.local_renderer
        if r.file_exists('/etc/init/homebot.conf'):
            r.sudo('rosrun robot_upstart uninstall {upstart_name}')

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
        r.sudo('rm -Rf /home/pi/.ros/log/*')

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
            '--exclude=db.sqlite3 '
            '--exclude=bags '
            '--exclude=.env '
            '--exclude=setup_local.bash '
            '--delete --rsh "ssh -t -o StrictHostKeyChecking=no -i {key_filename}" '
            'src {user}@{host_string}:{project_dir}')

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

    @task
    def init_log(self):
        """
        Initializes our custom log files.
        """
        r = self.local_renderer
        for log_path in self.lenv.log_paths:
            r.env.log_path = log_path
            if not r.file_exists(log_path):
                r.sudo('touch {log_path}; chown {user}:{group} {log_path}')

    @task(precursors=['packager', 'user', 'ros', 'rpi', 'ntp', 'ntpclient'])
    def configure(self):
        super(HomebotSatchel, self).configure() # runs tasks triggered by trackers

homebot = HomebotSatchel()
