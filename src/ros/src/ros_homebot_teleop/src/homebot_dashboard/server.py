from thread import start_new_thread
from time import sleep
from os import getpid, kill, environ
from signal import SIGINT

from django.conf import settings
from django.core.handlers.wsgi import WSGIHandler
# from django.core.management.commands.runserver import naiveip_re
from django.utils import six
from django.utils.autoreload import code_changed, restart_with_reloader
from socketio.server import SocketIOServer

DEFAULT_PORT = settings.DEFAULT_PORT
RELOAD = False

def reload_watcher():
    global RELOAD
    while True:
        RELOAD = code_changed()
        if RELOAD:
            kill(getpid(), SIGINT)
        sleep(5)

class Server(object):
    
    def __init__(self, addr='', port=DEFAULT_PORT, **kwargs):
        self.addr = ''
        self.port = port
        self.use_static_handler = kwargs.pop('use_static_handler', True)
        self.insecure_serving = kwargs.pop('insecure_serving', False)
        self.use_reloader = kwargs.pop('use_reloader', False)
        self.use_psyco = kwargs.pop('use_psyco', True)
        
        self.server = None

    def stop(self):
        self.server.stop()

    def get_handler(self):
        """
        Returns the django.contrib.staticfiles handler.
        """
        handler = WSGIHandler()
        try:
            from django.contrib.staticfiles.handlers import StaticFilesHandler
        except ImportError:
            return handler
        use_static_handler = self.use_static_handler
        insecure_serving = self.insecure_serving
        if (settings.DEBUG and use_static_handler or
                (use_static_handler and insecure_serving)):
            handler = StaticFilesHandler(handler)
        return handler
        
    def run(self):
        
#         if not addrport:
#             self.addr = ''
#             self.port = DEFAULT_PORT
#         else:
#             m = match(naiveip_re, addrport)
#             if m is None:
#                 raise CommandError('"%s" is not a valid port number '
#                                    'or address:port pair.' % addrport)
#             self.addr, _, _, _, self.port = m.groups()

        environ['DJANGO_SOCKETIO_PORT'] = str(self.port)

#         if options.get('use_psyco'):
#             try:
#                 from psycogreen.gevent import patch_psycopg
#             except ImportError:
#                 raise CommandError(
#                     'Could not patch psycopg. '
#                     'Is psycogreen installed?')
#             patch_psycopg()

        if self.use_reloader:
            start_new_thread(reload_watcher, ())

        try:
            bind = (self.addr, int(self.port))
            print 'SocketIOServer running on %s:%s\n\n' % bind
            handler = self.get_handler()
            server = self.server = SocketIOServer(
                bind, handler, resource='socket.io', policy_server=True)
            server.serve_forever()
        except KeyboardInterrupt:
            for key, sock in six.iteritems(server.sockets):
                sock.kill(detach=True)
            server.stop()
            if RELOAD:
                print 'Reloading...\n\n'
                restart_with_reloader()
