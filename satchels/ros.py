
from fabric.api import settings

from burlap.constants import *
from burlap import Satchel
from burlap.decorators import task

INDIGO = 'indigo'
KINETIC = 'kinetic'

class ROSSatchel(Satchel):

    name = 'ros'

    def set_defaults(self):

        # http://wiki.ros.org/Distributions
        #self.env.version_name = INDIGO # paired with Ubuntu 14 LTS
        self.env.version_name = KINETIC # paired with Ubuntu 16 LTS?

        #self.env.ubuntu_release = '$(lsb_release -sc)' # trusty for ARM

        self.env.conf_os_type = UBUNTU

        self.env.conf_os_release = None

        self.env.base_catkin_ws = '/home/{user}/ros_catkin_ws'

        self.env.rosinstall_generator_packages = []

        self.env.overlay_dir = None

        self.env.overlay_packages = []

        self.env.source_packages = []

        self.env.source_path = '/opt/ros/{version_name}/setup.bash'

        self.env.update_bash = True

        self.env.pip_packages = [
            'rosdep',
            'rosinstall_generator',
            'wstool',
            'rosinstall',
            #'wiringpi2==1.1.1',
            'wiringpi2==2.32.3',
        ]

    @task
    def clear_logs(self):
        r = self.local_renderer
        #r.sudo('rm -Rf ~/.ros/log/*')
        #http://wiki.ros.org/rosclean
        r.sudo('rosclean purge')

    @task
    def configure_ubuntu(self, reboot=1):
        """
        Installs ROS on Ubuntu.

        Based on instructions at:

            Ubuntu 16.05 Xenial Xerus

                http://wiki.ros.org/kinetic/Installation/Ubuntu
                http://wiki.ros.org/kinetic/Installation/UbuntuARM

            Ubuntu 14.04 Trusty Tahr

                http://wiki.ros.org/indigo/Installation/Ubuntu
                http://wiki.ros.org/indigo/Installation/UbuntuARM

                http://wiki.ros.org/jade/Installation/Ubuntu
                http://wiki.ros.org/jade/Installation/UbuntuARM

        """
        from burlap.packager import packager

        r = self.local_renderer

        # Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."
        # Note, this appears to be the default?
        # `cat /etc/apt/sources.list` shows these are already enabled on a default
        # Ubuntu ARM install.

        # Set your Locale
        # Boost and some of the ROS tools require that the system locale be set.
        # You can set it with:
        r.sudo('update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX')

        #self.install_repositories(service=self.name)
        packager.configure(service=self.name, initial_upgrade=0)

        # HANDLED BY PACKAGER
        # Setup your sources.list
        #r.sudo("sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu {ubuntu_release} main\" >
        # /etc/apt/sources.list.d/ros-latest.list'".format(**self.lenv))
        # HANDLED BY PACKAGER
        # Set up your keys
        #r.sudo("apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80
        # --recv-key 0xB01FA116")

        # HANDLED BY PACKAGER
        # First, make sure your Debian package index is up-to-date:
        #r.sudo('apt-get update')

        # HANDLED BY PACKAGER
        # Install base packages.
        #r.sudo('apt-get install --yes ros-%s-ros-base' % self.env.version_name)

        # HANDLED BY PACKAGER
        # Install base Python packages.
        #r.sudo('apt-get install --yes python-rosdep')

        # Initialize rosdep
        with settings(warn_only=True):
            r.sudo('rosdep init')
        r.run('rosdep update')

        # Environment setup
        # It's convenient if the ROS environment variables are automatically added to your bash
        # session every time a new shell is launched:
        if self.env.update_bash:
            r.run('echo "source /opt/ros/{version_name}/setup.bash" >> ~/.bash_aliases')
        #TODO:how to do this system-wide?
        #source ~/.bashrc

        if int(reboot):
            with settings(warn_only=True):
                self.reboot()

    @property
    def packager_repositories(self):
        if self.env.conf_os_type == UBUNTU:
            return {
                APT_SOURCE: [
                    ('deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main',
                        '/etc/apt/sources.list.d/ros-latest.list'),
                ],
                APT_KEY: [
                    ('hkp://ha.pool.sks-keyservers.net:80', '0xB01FA116'),
                ],
            }
        if self.env.conf_os_type == RASPBIAN:
            return {
                APT_SOURCE: [
                    ('deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main',
                        '/etc/apt/sources.list.d/ros-latest.list'),
                ],
                APT_KEY: [
                    #('hkp://ha.pool.sks-keyservers.net:80', '0xB01FA116'),
                    #wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
                    #-O - | sudo apt-key add -
                    'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key',
                ],
            }
        else:
            return {}

    @property
    def packager_system_packages(self):
        d = {
            UBUNTU: [
                'ros-%s-ros-base' % self.env.version_name,
                'python-rosdep',
                'ros-%s-xacro' % self.env.version_name,
                'ninja-build',
            ],
            RASPBIAN: [
                'python-pip',
                'python-setuptools',
                'python-yaml',
                'python-distribute',
                'python-docutils',
                'python-dateutil',
                'python-six',
                'python-serial',
                'libyaml-dev',
                # Needed by diagnostics and opencv3 when installing from source.
                'ninja-build',
            ],
        }
        return d

    @property
    def packager_locale(self):
        return {
            UBUNTU: dict(
                LANG='C',
                LANGUAGE='C',
                LC_ALL='C',
                LC_MESSAGES='POSIX',
            ),
        }

    @task
    def configure_raspbian_indigo(self):
        """
        Installs ROS on Debian/Raspbian.

        Based on instructions at:

            http://wiki.ros.org/ROSberryPi/Setting%20up%20ROS%20on%20RaspberryPi
            http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

        """
        raise NotImplementedError

#     @task
#     def install_wiringpi(self):
#         r = self.local_renderer
#         r.pc('Installing wiringpi2')
#         ret = r.run('pip show wiringpi2')
#         if not ret:
#             r.run('cd /tmp; git clone git://git.drogon.net/wiringPi')
#             r.sudo('cd /tmp/wiringPi; sudo ./build')

    @task
    def install_new_source_package(self, name):
        """
        Installs a new source package to an existing ROS system.
        """
        self.install_source_packages(names=name)
        self.build_source_packages()

    @task
    def install_source_packages(self, names=None):
        """
        Installs all source packages. Should only be run on a source installation.
        """
        r = self.local_renderer

        names = names or [package_name for package_name, _ in r.env.source_packages]
        if isinstance(names, basestring):
            names = [names]

        r.pc('Installing source packages.')
        for package_name in names:
            r.env.package_name = package_name
            r.run('cd {base_catkin_ws}; rosinstall_generator {package_name} '
                '--rosdistro {version_name} --deps | wstool merge -t src -')
            r.run('cd {base_catkin_ws}; wstool update -t src -j2 --delete-changed-uris')

    @task
    def build_source_packages(self):
        """
        Compiles packages from source. Should be run after install_source_packages().
        """
        r = self.local_renderer
        r.run('cd {base_catkin_ws}; rosdep install --from-paths src --ignore-src '
            '--rosdistro {version_name} -y -r --os=debian:jessie')
        r.sudo('cd {base_catkin_ws}; ./src/catkin/bin/catkin_make_isolated --install '
            '-DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/{version_name} -j1')

    @task
    def install_source_packages_apt(self, version_name=''):
        r = self.local_renderer
        r.env.version_name = version_name or r.env.version_name
        r.env.packages = ' '.join([
            'ros-' + r.env.version_name + '-' + package_name.replace('_', '-')
            for package_name, checkout_command in r.env.source_packages])
        r.sudo('apt-get install -y {packages}')

    @task
    def install_overlay_workspace(self, clean=0):
        """
        Initializes the ROS overlay of packages we need to compile from source that
        don't have apt packages but aren't directly included in Homebot.
        """
        clean = int(clean)
        r = self.local_renderer
        if not r.env.overlay_dir or not r.env.overlay_packages:
            return

        if clean:
            r.sudo('rm -Rf {overlay_dir}')
        r.sudo('[ ! -d "{overlay_dir}/src" ] && mkdir -p {overlay_dir}/src || true')
        r.sudo('chown -R {user}:{user} {overlay_dir}/..')

        for package_name, checkout_command in r.env.overlay_packages:
            r.env.package_name = package_name
            r.env.checkout_command = checkout_command
            r.run('cd {overlay_dir}/src; [ ! -d {package_name} ] && {checkout_command} || true')

        r.run('source /opt/ros/{version_name}/setup.bash; cd {overlay_dir}; catkin_make')

    @task
    def configure_raspbian_kinetic(self):
        """
        Installs ROS on Debian/Raspbian.

        Based on instructions at:

            http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
            http://dev.px4.io/ros-raspberrypi-installation.html
            http://wiki.ros.org/kinetic/Installation/Source

        """

        r = self.local_renderer

        r.pc('Installing global pip dependencies')
        r.env.pip_packages = ' '.join(r.env.pip_packages)
        r.sudo('pip install {pip_packages}')

        r.pc('Initialize rosdep')
        with settings(warn_only=True):
            r.sudo('rosdep init')
        r.run('rosdep update')

        r.pc('Create and build catkin workspace')
        r.run('[ ! -d "{base_catkin_ws}" ] && mkdir -p {base_catkin_ws} || true')
        r.env.package_names = ' '.join(['ros_comm'] \
            + [package_name for package_name, _ in r.env.source_packages])
        r.run('cd {base_catkin_ws}; rosinstall_generator {package_names} '
            '--rosdistro {version_name} --deps --wet-only --exclude roslisp --tar '
            '> {version_name}-ros_comm-wet.rosinstall')
        r.sudo('[ -d "{base_catkin_ws}/src" ] && rm -Rf {base_catkin_ws}/src || true')
        r.run('cd {base_catkin_ws}; wstool init src {version_name}-ros_comm-wet.rosinstall')

#         self.install_source_packages()

        #TODO:build collada-dom-dev?

        r.pc('Installing system dependencies based on desired ROS packages.')
        r.run('cd {base_catkin_ws}; rosdep install --from-paths src --ignore-src '
            '--rosdistro {version_name} -y -r --os=debian:{conf_os_release}')

        # If this runs out of memory, try increasing Swap.
        # http://raspberrypimaker.com/adding-swap-to-the-raspberrypi/
        # Note, -j1 is required, otherwise the build fails at cv_bridge, which consumes
        # too much memory.
        # Takes about 75 minutes.
        r.sudo('cd {base_catkin_ws}; ./src/catkin/bin/catkin_make_isolated --install '
            '-DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/{version_name} -j1')

        #r.run('source {base_catkin_ws}/install_isolated/setup.bash')
        r.run('source /opt/ros/{version_name}/setup.bash')

        self.install_overlay_workspace()

        r.append(
            #text="source /opt/ros/{version_name}/setup.bash".format(**r.env),
            text="source {source_path}".format(**r.env),
            filename='~/.bash_aliases')

#         self.reboot()

    @task(precursors=['packager', 'user', 'timezone', 'sshnice', 'rpi', 'avahi', 'nm', 'ntpclient'])
    def configure(self):
        if self.env.conf_os_type == UBUNTU:
            self.configure_ubuntu()
        elif self.env.conf_os_type == RASPBIAN:
            # Warning, Raspiban support is experimental!
            if self.env.conf_os_release == JESSIE:
                if self.env.version_name == INDIGO:
                    self.configure_raspbian_indigo()
                elif self.env.version_name == KINETIC:
                    self.configure_raspbian_kinetic()
            else:
                raise NotImplementedError, \
                    'Unsupported Raspbian ROS release: %s' % self.env.conf_os_release
        else:
            raise NotImplementedError, 'Unsupported OS: %s' % self.env.conf_os_type

ros = ROSSatchel()
