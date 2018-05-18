import unittest
import time
import re


class State(object):
    """
    Helper object to keep track of message values as a key/value lookup.
    """

    def __init__(self):
        self._values = {} # {name: value}
        self._times_last_updated = {} # {name: time last updated}
        self._times_to_stale = {}

    def clean_name(self, name, validate=True):
        name = re.sub(r'/+', '_', name)
        name = re.sub(r'^_+', '', name)
        if validate:
            assert name in self._values, 'Unknown state key "%s".' % name
        return name

    def register(self, name, time_to_stale=0, default=None):
        """
        Formally declares a variable name to track.
        """
        name = self.clean_name(name, validate=False)
        self._values.setdefault(name, default)
        self._times_last_updated.setdefault(name, 0)
        self._times_to_stale.setdefault(name, time_to_stale)

    def __getattr__(self, name):
        try:
            return super(State, self).__getattribute__(name)
        except AttributeError:
            if name in self._values:
                if self._times_to_stale[name]:
                    if time.time() - self._times_last_updated[name] <= self._times_to_stale[name]:
                        # If the value can go stale, only return a value if it's still fresh.
                        return self._values[name]
                    else:
                        # Key is stale, so return nothing.
                        return
                # Otherwise, value never goes stale, so return what we have.
                return self._values[name]
            raise

    def get(self, name):
        name = self.clean_name(name)
        return getattr(self, name)

    def get_since(self, name, t=0):
        """
        Returns a value only if its last-updated timestamp is equal to or greater than the given timestamp.
        """
        name = self.clean_name(name)
        if self._times_last_updated[name] >= t:
            return getattr(self, name)

    def set(self, name, value):
        name = self.clean_name(name)
        self._values[name] = value
        self._times_last_updated[name] = time.time()

class Test(unittest.TestCase):

    def test_state(self):
        s = State()
        # Init keys.
        s.register('edge0')
        s.register('/torso_arduino/motor_target_a')
        s.register('qr_image', time_to_stale=1)
        # Verify simple key.
        self.assertEqual(s.torso_arduino_motor_target_a, None)
        self.assertEqual(s.edge0, None)
        s.set('/torso_arduino/motor_target_a', 4)
        self.assertEqual(s.get('/torso_arduino/motor_target_a'), 4)
        s.set('edge0', 1)
        self.assertEqual(s.edge0, 1)
        self.assertEqual(s.torso_arduino_motor_target_a, 4)
        s.set('edge0', 0)
        self.assertEqual(s.edge0, 0)
        # Verify stale key.
        self.assertEqual(s.qr_image, None)
        s.set('qr_image', 'abc')
        self.assertEqual(s.qr_image, 'abc')
        self.assertEqual(s.get('qr_image'), 'abc')
        time.sleep(2)
        # We've waiting too long and now the value has gone stale.
        self.assertEqual(s.qr_image, None)
        s.set('qr_image', 'xyz')
        t0 = time.time()
        self.assertEqual(s.qr_image, 'xyz')
        self.assertEqual(s.get('qr_image'), 'xyz')
        # We recorded our reference time-stamp before we last set this, so get_since() should return nothing.
        self.assertEqual(s.get_since('qr_image', t0), None)
        s.set('qr_image', 'xyz')
        # Now that we've updated it more recently, get_since() should return something.
        self.assertEqual(s.get_since('qr_image', t0), 'xyz')

if __name__ == '__main__':
    unittest.main()
