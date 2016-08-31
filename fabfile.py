#.env/bin/python
"""
Homebot management script.
"""

from fabric.api import (
    task, env, local, run, sudo, get, put, runs_once, execute, settings, task
)

from burlap.common import (
    put_or_dryrun,
    get_or_dryrun,
    sudo_or_dryrun,
    local_or_dryrun,
    run_or_dryrun,
    add_deployer,
    service_configurators,
    manifest_recorder,
    add_deployer,
)
from burlap.decorators import task_or_dryrun

from satchels import ros, rpi, homebot, arduino

@task_or_dryrun
def deploy_cura():
    """
    Updates files for the Printrbot manager. e.g.
    
        fab printer deploy_cura
    """
    
    # Ensure our 3d configuration options are up-to-date.
    run_or_dryrun('mkdir -p ~/git; cd ~/git; git clone https://github.com/chrisspen/3d-printer-profiles.git; cd 3d-printer-profiles; git pull')
    
    # Ensure our 3d models are up-to-date.
    sudo_or_dryrun('mkdir -p %(project_home)s/models/printable' % env)
    sudo_or_dryrun('chown -R %(user)s:%(user)s %(project_home)s' % env)
    local_or_dryrun('rsync -avz --delete --rsh "ssh -t -o StrictHostKeyChecking=no -i %(key_filename)s" models/printable %(user)s@%(host_string)s:%(project_home)s/models/' % env)

#DEPRECATED
@task_or_dryrun
def torso_tester():
    shell(command='cd /usr/local/homebot/src; . ./shell; cd ros/ros_homebot_arduino_python/src/ros_homebot_arduino_python/bin; ./torso_arduino_tester.py')
    #Really sluggish high latency.
    #run_or_dryrun('cd /usr/local/homebot/src; . ./shell; cd ros/ros_homebot_arduino_python/src/ros_homebot_arduino_python/bin; ./torso_arduino_tester.py')

#DEPRECATED
@task_or_dryrun
def head_tester():
    shell(command='cd /usr/local/homebot/src; . ./shell; cd ros/ros_homebot_arduino_python/src/ros_homebot_arduino_python/bin; ./head_arduino_tester.py')
