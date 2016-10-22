import re

from ros_homebot_python import constants as c
from ros_homebot_python import utils

def camelcase_to_underscores(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def hash_str(s):
#     print 'hash str:', repr(s)
    s = str(s)
    return sum(ord(letter) for letter in s)

class Packet(object):
    
    def __init__(self, id, data='', check_id=True): # pylint: disable=W0622
        if isinstance(id, int):
            id = chr(id)
        self.id = str(id)
        if check_id:
            assert self.id in c.ALL_IDS, 'Invalid ID: %s' % self.id
        self.data = str(data)
        
    @property
    def hash(self):
        return hash_str(self.id + self.data)
    
    @classmethod
    def from_ros_message(cls, msg):
        return cls(chr(msg.id), msg.data)
    
    @property
    def parameters(self):
        return self.data.split(' ')
    
    @property
    def id_name(self):
        if self.id not in c.ALL_IDS:
            return
        return re.sub(r'[^a-z]+', '_', c.ALL_IDS[self.id].lower().strip())
    
    @property
    def length(self):
        return len(self.id.strip()) + len(self.data.strip())
    
#     @classmethod
#     def fromServiceRequest(cls, req, packet_id):
#         req_name = type(req).__name__
        #packet_name = camelcase_to_underscores(req_name)
    
    @classmethod
    def from_string(cls, s):
        _id = c.ID_NULL
        _data = ''
        if s:
            parts = s.split(' ')
            if parts:
                _id = parts[0]
                _data = ' '.join(parts[1:])
        return cls(_id, _data, check_id=False)
        
    def __unicode__(self):
        s = u'%s %s' % (self.id_name, self.data)
        s = s.strip()
        s = s.encode('utf-8')
        return s
        
    def __repr__(self):
        return u'<%s: %s>' % (type(self).__name__, unicode(self))

class KVPacket(Packet):
    
    def __init__(self, id, *args, **kwargs): # pylint: disable=W0622
        data = ' '.join(map(str, args))
        super(KVPacket, self).__init__(id, data, **kwargs)
        
class BooleanPacket(Packet):
    """
    A packet with a single parameter that is either true or false.
    """
    
    def __init__(self, id, data='', **kwargs): # pylint: disable=W0622
        data = utils.to_10(data)
        super(BooleanPacket, self).__init__(id, **kwargs)
        self.data = data
        
class LEDPacket(BooleanPacket):
    
    def __init__(self, data=''):
        super(LEDPacket, self).__init__(c.ID_LED, data)
