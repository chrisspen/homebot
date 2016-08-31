"""
Raspberry Pi
===================

Tools for configuring software and features specific to a Raspberry Pi.
"""
import os
import getpass

from burlap import Satchel
from burlap.constants import *
from burlap.decorators import task

RPI2 = 'rpi2'
RPI3 = 'rpi3'

class RaspberryPiSatchel(Satchel):
    """
    Tools for configuring the Raspberry Pi.
    
    Most of these were written for the Raspberry Pi 2, but now target the Raspberry Pi 3.
    """

    name = 'rpi'
    
    def set_defaults(self):
        self.env.i2c_enabled = False
        self.env.gpio_enabled = False
        self.env.camera_enabled = False
        
        self.env.hardware_version = RPI3
        
        self.env.firmware_update_bin_url = 'https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update'
        
        # The SD card reader and mount info.
        self.env.sd_device = '/dev/sdb'
        self.env.sd_media_mount_dir = '/dev/sdb1'
        self.env.sd_media_mount_dir2 = '/dev/sdb2'
        
        # Raspbian specifics.
        # Should be one of the filenames found at:
        # https://downloads.raspberrypi.org
        self.env.raspbian_image_zip = 'raspbian_lite_latest.zip'
        self.env.raspbian_download_url = 'https://downloads.raspberrypi.org/raspbian_lite_latest'
        self.env.raspbian_mount_point = '/mnt/img'
        # Should be one of the filenames found at:
        # https://github.com/dhruvvyas90/qemu-rpi-kernel
        self.env.raspbian_kernel = 'kernel-qemu-4.1.13-jessie'
        
        # Ubuntu specifics.
        self.env.ubuntu_download_url = 'http://www.finnie.org/software/raspberrypi/ubuntu-rpi3/ubuntu-16.04-preinstalled-server-armhf+raspi3.img.xz'
        
        self.env.conf_os_type = RASPBIAN
        self.env.conf_os_release = JESSIE
        
        self.env.libvirt_boot_dir = '/var/lib/libvirt/boot'
        self.env.libvirt_images_dir = '/var/lib/libvirt/images'
        
#         self.env.default_hostname = 'raspberrypi'
#         self.env.default_user = 'pi'
#         self.env.default_password = 'raspberry'

    @task
    def update_firmware(self):
        r = self.local_renderer
        r.pc('Updating firmware.')
        
        packager = self.packager
        if packager == APT:
            r.sudo('apt-get install -y binutils')
        else:
            raise NotImplementedError
        
        # Install most recent version of rpi-update, if not present.
        r.sudo("[ ! -f '/usr/bin/rpi-update' ] && curl -L --output /usr/bin/rpi-update {firmware_update_bin_url} && chmod +x /usr/bin/rpi-update || true")
        
        # Update firmware.
        r.sudo("sudo rpi-update")
        
        # Reboot to take effect.
        self.reboot(wait=300, timeout=60)

    def assume_localhost(self):
        """
        Sets connection parameters to localhost, if not set already.
        """
        if not self.genv.host_string:
            self.genv.host_string = 'localhost'
            self.genv.hosts = ['localhost']
            self.genv.user = getpass.getuser()

    @task
    def init_raspbian_disk(self, yes=0):
        """
        Downloads the latest Raspbian image and writes it to a microSD card.
        
        Based on the instructions from:
        
        https://www.raspberrypi.org/documentation/installation/installing-images/linux.md
        """
        self.assume_localhost()
        
        yes = int(yes)
        device_question = 'SD card present at %s? ' % self.env.sd_device
        if not yes and not raw_input(device_question).lower().startswith('y'):
            return
            
        r = self.local_renderer
        r.local('[ ! -f {raspbian_image_zip} ] && wget {raspbian_download_url} -O raspbian_lite_latest.zip || true')
            
        r.lenv.img_fn = r.local("unzip -l {raspbian_image_zip} | sed -n 4p | awk '{{print $4}}'", capture=True) or '$IMG_FN'
        r.local('echo {img_fn}')
        r.local('[ ! -f {img_fn} ] && unzip {raspbian_image_zip} {img_fn} || true')
        r.lenv.img_fn = r.local('readlink -f {img_fn}', capture=True)
        r.local('echo {img_fn}')
        
        with self.settings(warn_only=True):
            r.sudo('[ -d "{sd_media_mount_dir}" ] && umount {sd_media_mount_dir} || true')
        with self.settings(warn_only=True):
            r.sudo('[ -d "{sd_media_mount_dir2}" ] && umount {sd_media_mount_dir2} || true')
            
        r.pc('Writing the image onto the card.')
        r.sudo('time dd bs=4M if={img_fn} of={sd_device}')
        
        # Flush all writes to disk.
        r.run('sync')

    @task
    def init_ubuntu_disk(self, yes=0):
        """
        Downloads the latest Ubuntu image and writes it to a microSD card.
        
        Based on the instructions from:
        
            https://wiki.ubuntu.com/ARM/RaspberryPi
        
        For recommended SD card brands, see:
        
            http://elinux.org/RPi_SD_cards
        
        """
        self.assume_localhost()
        
        yes = int(yes)
        device_question = 'SD card present at %s? ' % self.env.sd_device
        if not yes and not raw_input(device_question).lower().startswith('y'):
            return
        
        r = self.local_renderer
        
        # Confirm SD card is present.
        r.local('ls {sd_device}')
        
        # Download image.
        r.env.ubuntu_image_fn = os.path.abspath(os.path.split(self.env.ubuntu_download_url)[-1])
        r.local('[ ! -f {ubuntu_image_fn} ] && wget {ubuntu_download_url} || true')
        
        # Ensure SD card is unmounted.
        with self.settings(warn_only=True):
            r.sudo('[ -d "{sd_media_mount_dir}" ] && umount {sd_media_mount_dir}')
        with self.settings(warn_only=True):
            r.sudo('[ -d "{sd_media_mount_dir2}" ] && umount {sd_media_mount_dir2}')
        
        r.pc('Writing the image onto the card.')
        r.sudo('xzcat {ubuntu_image_fn} | dd bs=4M of={sd_device}')
        
        # Flush all writes to disk.
        r.run('sync')

    #EXPERIMENTAL
    @task
    def init_raspbian_vm(self):
        """
        Creates an image for running Raspbian in a QEMU virtual machine.
        
        Based on the guide at:
        
            https://github.com/dhruvvyas90/qemu-rpi-kernel/wiki/Emulating-Jessie-image-with-4.1.x-kernel
        """
        
        r = self.local_renderer
        
        r.comment('Installing system packages.')
        r.sudo('add-apt-repository ppa:linaro-maintainers/tools')
        r.sudo('apt-get update')
        r.sudo('apt-get install libsdl-dev qemu-system')
        
        r.comment('Download image.')
        r.local('wget https://downloads.raspberrypi.org/raspbian_lite_latest')
        r.local('unzip raspbian_lite_latest.zip')
        #TODO:fix name?
        #TODO:resize image?
        
        r.comment('Find start of the Linux ext4 partition.')
        r.local("parted -s 2016-03-18-raspbian-jessie-lite.img unit B print | awk '/^Number/{{p=1;next}}; p{{gsub(/[^[:digit:]]/, "", $2); print $2}}' | sed -n 2p", assign_to='START')
        
        r.local('mkdir -p {raspbian_mount_point}')
        r.sudo('mount -v -o offset=$START -t ext4 {raspbian_image} $MNT')
        
        r.comment('Comment out everything in ld.so.preload')
        r.local("sed -i 's/^/#/g' {raspbian_mount_point}/etc/ld.so.preload")
        
        r.comment('Comment out entries containing /dev/mmcblk in fstab.')
        r.local("sed -i '/mmcblk/ s?^?#?' /etc/fstab")
        
        r.sudo('umount {raspbian_mount_point}')
        
        r.comment('Download kernel.')
        r.local('wget https://github.com/dhruvvyas90/qemu-rpi-kernel/blob/master/{raspbian_kernel}?raw=true')
        r.local('mv {raspbian_kernel} {libvirt_images_dir}')
        
        r.comment('Creating libvirt machine.')
        r.local('virsh define libvirt-raspbian.xml')
        
        r.comment('You should now be able to boot the VM by running:')
        r.comment('')
        r.comment('    qemu-system-arm -kernel {libvirt_boot_dir}/{raspbian_kernel} -cpu arm1176 -m 256 -M versatilepb -serial stdio -append "root=/dev/sda2 rootfstype=ext4 rw" -hda {libvirt_images_dir}/{raspbian_image}')
        r.comment('')
        r.comment('Or by running virt-manager.')
    
    @task
    def create_raspbian_vagrant_box(self):
        """
        Creates a box for easily spinning up a virtual machine with Vagrant.
        
        http://unix.stackexchange.com/a/222907/16477
        https://github.com/pradels/vagrant-libvirt
        """
        
        r = self.local_renderer
        
        r.sudo('adduser --disabled-password --gecos "" vagrant')
        
        #vagrant user should be able to run sudo commands without a password prompt
        
        r.sudo('echo "vagrant ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vagrant')
        r.sudo('chmod 0440 /etc/sudoers.d/vagrant')
        
        r.sudo('apt-get update')
        r.sudo('apt-get install -y openssh-server')
        
        #put ssh key from vagrant user
        
        r.sudo('mkdir -p /home/vagrant/.ssh')
        r.sudo('chmod 0700 /home/vagrant/.ssh')
        r.sudo('wget --no-check-certificate https://raw.github.com/mitchellh/vagrant/master/keys/vagrant.pub -O /home/vagrant/.ssh/authorized_keys')
        r.sudo('chmod 0600 /home/vagrant/.ssh/authorized_keys')
        r.sudo('chown -R vagrant /home/vagrant/.ssh')
        
        #open sudo vi /etc/ssh/sshd_config and change
        
        #PubKeyAuthentication yes
        #PermitEmptyPasswords no
        r.sudo("sed -i '/AuthorizedKeysFile/s/^#//g' /etc/ssh/sshd_config")
        #PasswordAuthentication no
        r.sudo("sed -i '/PasswordAuthentication/s/^#//g' /etc/ssh/sshd_config")
        r.sudo("sed -i 's/PasswordAuthentication yes/PasswordAuthentication no/g' /etc/ssh/sshd_config")
        
        #restart ssh service using
        
        #sudo service ssh restart
        
        #install additional development packages for the tools to properly compile and install
        r.sudo('apt-get upgrade')
        r.sudo('apt-get install -y gcc build-essential')
        #TODO:fix? throws dpkg: error: fgets gave an empty string from `/var/lib/dpkg/triggers/File'
        #r.sudo('apt-get install -y linux-headers-rpi')
        
        #do any change that you want and shutdown the VM . now , come to host machine on which guest VM is running and goto the /var/lib/libvirt/images/ and choose raw image in which you did the change and copy somewhere for example /test
        
        r.sudo('mkdir /tmp/test')
        r.sudo('cp {libvirt_images_dir}/{raspbian_image} /tmp/test')
        r.sudo('cp {libvirt_boot_dir}/{raspbian_kernel} /tmp/test')
        
        #create two file metadata.json and Vagrantfile in /test do entry in metadata.json
        r.render_to_file('rpi/metadata.json', '/tmp/test/metadata.json')
        r.render_to_file('rpi/Vagrantfile', '/tmp/test/Vagrantfile')
        
        #convert test.img to qcow2 format using
        r.sudo('qemu-img convert -f raw -O qcow2  {libvirt_images_dir}/{raspbian_image}  {libvirt_images_dir}/{raspbian_image}.qcow2')
        
        #rename ubuntu.qcow2 to box.img
        r.sudo('mv {libvirt_images_dir}/{raspbian_image}.qcow2 {libvirt_images_dir}/box.img')
        
        #Note: currently,libvirt-vagrant support only qcow2 format. so , don't change the format just rename to box.img. because it takes input with name box.img by default.
        #create box
        
        r.sudo('cd /tmp/test; tar cvzf custom_box.box ./metadata.json ./Vagrantfile ./{raspbian_kernel} ./box.img') 
        
        #add box to vagrant
        
        #vagrant box add --name custom custom_box.box
        
        #go to any directory where you want to initialize vagrant and run command bellow that will create Vagrant file
        
        #vagrant init custom
        
        #start configuring vagrant VM
        
        #vagrant up --provider=libvirt 
        #TODO:fix? Error while creating domain: Error saving the server: Call to virDomainDefineXML failed: XML error: No PCI buses available

    @property
    def packager_repositories(self):
        d = {}
        # Recommended by https://wiki.ubuntu.com/ARM/RaspberryPi
        if self.env.conf_os_type == UBUNTU:
            if self.env.hardware_version == RPI2:
                d[APT] = ['ppa:ubuntu-raspi2/ppa']
            elif self.env.hardware_version == RPI3:
                d[APT] = ['ppa:ubuntu-raspi2/ppa-rpi3']
        return d

    @property
    def packager_system_packages(self):
        UBUNTU_lst = ['curl']
        DEBIAN_lst = ['curl']
        RASPBIAN_lst = ['curl', 'rpi-update']
        
        if self.env.i2c_enabled:
            UBUNTU_lst.extend(['python-smbus', 'i2c-tools', 'git', 'python-dev', 'libi2c-dev'])
            DEBIAN_lst.extend(['python-smbus', 'i2c-tools', 'git', 'python-dev', 'libi2c-dev'])
            RASPBIAN_lst.extend(['python-smbus', 'i2c-tools', 'git', 'python-dev', 'libi2c-dev'])
            
        return {
            UBUNTU: UBUNTU_lst,
            DEBIAN: DEBIAN_lst,
            RASPBIAN: RASPBIAN_lst,
        }
        
    @task
    def test_i2c(self):
        r = self.local_renderer
        #https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c
        r.sudo('i2cdetect -y 1')
        
    @task
    def configure_camera(self):
        """
        Enables access to the camera.
        
            http://raspberrypi.stackexchange.com/questions/14229/how-can-i-enable-the-camera-without-using-raspi-config
            https://mike632t.wordpress.com/2014/06/26/raspberry-pi-camera-setup/
        """
        r = self.local_renderer
        if self.env.camera_enabled:
            r.pc('Enabling camera.')
            
            # Set start_x=1
            r.run('if grep "start_x=0" /boot/config.txt; then sed -i "s/start_x=0/start_x=1/g" /boot/config.txt; fi')
            r.sudo('if grep "start_x" /boot/config.txt; then true; else echo "start_x=1" >> /boot/config.txt; fi')
            
            # Set gpu_mem=128
            r.sudo('if grep "gpu_mem" /boot/config.txt; then true; else echo "gpu_mem=128" >> /boot/config.txt; fi')
            
            # Compile the Raspberry Pi binaries.
            #https://github.com/raspberrypi/userland
            r.run('cd /tmp; git clone https://github.com/raspberrypi/userland.git; cd userland; ./buildme')
            r.run('touch ~/.bash_aliases')
            #r.run("echo 'PATH=$PATH:/opt/vc/bin\nexport PATH' >> ~/.bash_aliases")
            r.append(r'PATH=$PATH:/opt/vc/bin\nexport PATH', '~/.bash_aliases')
            #r.run("echo 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib\nexport LD_LIBRARY_PATH' >> ~/.bash_aliases")
            r.append(r'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib\nexport LD_LIBRARY_PATH', '~/.bash_aliases')
            r.run('source ~/.bashrc')
            r.sudo('ldconfig')
        
            # Allow our user to access the video device.
            r.sudo("echo 'SUBSYSTEM==\"vchiq\",GROUP=\"video\",MODE=\"0660\"' > /etc/udev/rules.d/10-vchiq-permissions.rules")
            r.sudo("usermod -a -G video {user}")
            
            r.reboot(wait=300, timeout=60)
            
        else:
            pass

    @task
    def configure_gpio(self):
        r = self.local_renderer
        if self.env.gpio_enabled:
            r.pc('Enabling GPIO.')
            
            # assumes init_project ran first
            r.sudo("usermod -a -G gpio {user}")
            #sudo chown root:gpio /sys/class/gpio/unexport /sys/class/gpio/export
            #sudo chmod 220 /sys/class/gpio/unexport /sys/class/gpio/export
            
            # Make GPIO accessible to non-root users.
            #Obsolete in Ubuntu 16?
            #r.sudo("echo 'SUBSYSTEM==\"gpio*\", PROGRAM=\"/bin/sh -c 'chown -R root:gpio /sys/class/gpio && chmod -R 770 /sys/class/gpio; chown -R root:gpio /sys/devices/virtual/gpio && chmod -R 770 /sys/devices/virtual/gpio'\"' > /etc/udev/rules.d/99-com.rules")
        else:
            pass

    @task
    def configure_i2c(self):
        #TODO:fix? causes RPi3 to become unbootable?
        r = self.local_renderer
        if self.env.i2c_enabled:
            r.pc('Enabling I2C.')
            
            #r.sudo('apt-get install --yes python-smbus i2c-tools git python-dev')
            
#             r.sudo("sh -c 'echo \"i2c-bcm2708\" >> /etc/modules'")
#             r.sudo("sh -c 'echo \"i2c-dev\" >> /etc/modules'")
#             r.sudo("sh -c 'echo \"dtparam=i2c1=on\" >> /boot/config.txt'")
#             r.sudo("sh -c 'echo \"dtparam=i2c_arm=on\" >> /boot/config.txt'")
            
            # Allow non-root users to access I2C.
            # http://quick2wire.com/non-root-access-to-spi-on-the-pi/
            # https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-i2c.md
            # https://blogs.ncl.ac.uk/francisfranklin/2014/03/23/using-i2c-with-the-raspberry-pi-step-1-modules-and-packages/
            r.sudo('groupadd -f --system spi')
            r.sudo('adduser {user} spi')
            r.sudo('adduser {user} i2c')
            r.append(text='SUBSYSTEM=="spidev", GROUP="spi"', filename='/etc/udev/rules.d/90-spi.rules', use_sudo=True)
            r.append(text='SUBSYSTEM=="i2c-dev", MODE="0666"', filename='/etc/udev/rules.d/99-i2c.rules', use_sudo=True)
            r.append(text='KERNEL=="i2c-[0-7]",MODE="0666"', filename='/etc/udev/rules.d/90-i2c.rules', use_sudo=True) 

            r.append(text='i2c-bcm2708', filename='/etc/modules', use_sudo=True)
            r.append(text='i2c-dev', filename='/etc/modules', use_sudo=True)
            r.append(text='dtparam=i2c1=on', filename='/boot/config.txt', use_sudo=True)
            r.append(text='dtparam=i2c_arm=on', filename='/boot/config.txt', use_sudo=True)
            
            r.reboot(wait=300, timeout=60)
            
            # If I2C is working, running this should show addresses in use.
            ret = r.sudo('i2cdetect -y 1')
            if not self.dryrun:
                assert ret, 'I2C configuration failed!'
        else:
            pass
            
    @task
    def configure(self):
        self.update_firmware()
        self.configure_i2c()
        self.configure_camera()
        self.configure_gpio()
        
    configure.deploy_before = ['packager', 'user', 'timezone', 'arduino', 'avahi', 'nm', 'ntpclient', 'sshnice']

RaspberryPiSatchel()
