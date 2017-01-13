
from django.contrib.auth.decorators import login_required
from django.shortcuts import render
# from django_socketio import broadcast, broadcast_channel, NoSocket

from ros_homebot_python import constants as c

@login_required
def index(request, template="index.html"):
    """
    Homepage
    """
    context = {
        'host': request.META['HTTP_HOST'].split(':')[0],
        'video_port': 8181,
    }
    context.update(
        (k, v)
        for k, v in c.__dict__.iteritems()
        if k.startswith('ID_') or k.startswith('MOTOR_') \
        or k in ('HEAD', 'TORSO') or k.startswith('TILT_')
    )
    return render(request, template, context)
