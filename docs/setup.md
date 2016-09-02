Software Setup
==============

Structure
---------

Configuring your robot will require two computers. The first is the Raspberry Pi physically located on the robot. The second is the deployment machine, where the robot's base code and configuration will be managed from and eventually deployed to the Raspberry Pi. Both computers are assumed to be running Linux.

Configuring the Deployment Machine
----------------------------------

Assuming your deployment machine is one you're reading this documentation from, your localhost, start by checking out this project via git. Open a terminal in a directory where you keep your Git projects and run:

    git clone https://github.com/chrisspen/homebot.git

Initialize your deployment environment by running:

    ./bootstrap

This command creates a Python virtual environment and installs a few required packages.

Note, any command presented in this guide beginning with "fab" should be run in a terminal from the project's root directory. Also, that terminal must first have your virtual environment activated by running:

    . ./setup.bash

Set your custom settings by running:

    cp roles/local/settings_local.yaml.template roles/local/settings_local.yaml

By default, it's assumed your deployment machine is running Ubuntu 16.04. Edit this file to select a different OS.
    
Next, install ROS by running:

    . ./setup.bash
    fab local ros.configure

New Install Checklist
---------------------

Decide on the hostname for your robot. For all intents and purposes, the robot will function like a Linux server

1. Ensure you've purged the domain name from your local router's DHCP cache, or it could interfere with the install.

    $ ping <domain>
    ping: unknown host <domain>
    
2. Purge all old SSH keys:

     fab prod host.purge_keys

Installing the Operating System
-------------------------------

1. Set your custom settings by running:

    cp roles/prod/settings_local.yaml.template roles/prod/settings_local.yaml
    
This file represents your custom software configurations to the robot and will override any defaults.

Edit the values in this file to suite your preferences. At the very least, you'll need to specify your wifi SSID and password so you can access the robot remotely.

2. Insert an SD card into the card reader on your deployment machine and run:

    fab local rpi.init_raspbian_disk
    
3. Insert card into Pi and power on.

4. Begin installation using:

    fab prod deploy.run
    
Note, this may take up to 5 hours to complete.
