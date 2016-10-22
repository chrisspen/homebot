#!/usr/bin/env python

import os

from gevent import monkey
#monkey.patch_all()

# from psycogreen.gevent import patch_psycopg

#os.environ.setdefault("DJANGO_SETTINGS_MODULE", "homebot_dashboard.settings")
# patch_psycopg()

from django.core.wsgi import get_wsgi_application

from socketio.server import SocketIOServer

monkey.patch_all()
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "homebot_dashboard.settings")
application = get_wsgi_application()

if __name__ == '__main__':
    server = SocketIOServer(('', 8000), application, resource="socket.io")
    server.serve_forever()
