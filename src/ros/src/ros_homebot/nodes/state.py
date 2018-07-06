"""
Test with:

    python state.py Test.test_trigger

"""
from __future__ import print_function
import unittest
import time
import re
import uuid
# from functools import partial
from threading import RLock


class TriggerTimeout(Exception):
    pass


class TriggerHandle(object):
    """
    Wraps management routines for inspecting a state trigger.
    """

    def __init__(self, tid, callback, state):
        self.tid = tid
        self.callback = callback
        self.state = state

    def __call__(self):
        return self.callback(tid=self.tid)

    def destroy(self):
        if not self.tid:
            raise Exception('Trigger has already been destroyed.')
        self.state.unset_trigger(self.tid)
        self.callback = None
        self.tid = None

    def wait(self, timeout=10, rate=0.5):
        """
        Blocks until the trigger or timeout occurs.
        """
        t0 = time.time()
        while not timeout or (time.time() - t0 <= timeout):
            if self():
                return
            time.sleep(rate)
        raise TriggerTimeout


class State(object):
    """
    Thread-safe helper object to keep track of message values as a key/value lookup.
    """

    def __init__(self):
        self._values = {} # {name: value}
        self._times_last_updated = {} # {name: time last updated}
        self._times_to_stale = {}
        self._lock = RLock()
        self._or_trigger_callables = {} # {tid: callable}
        self._or_trigger_results = {} # {tid: callable}
        self._or_trigger_dependencies = {} # {tid: [tid]}

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
            with self._lock:
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

    def get_since_until(self, name, t=0, timeout=10):
        """
        Like get_since(), but retries for `timeout` seconds.
        """
        t0 = time.time()
        while time.time() - t0 <= timeout:
            value = self.get_since(name=name, t=t)
            if value is not None:
                return value
            time.sleep(0.5)
        raise Exception('Value "%s" not seen in %i seconds.' % (name, timeout))

    def has_triggered(self, tid):
        """
        Returns true if the given trigger has activated, without removing it.
        """
        # return tid not in self._or_trigger_callables
        return tid in self._or_trigger_results

    def set(self, name, value, verbose=False):
        # if verbose: print('setting %s=%s' % (name, value))
        with self._lock:
            name = self.clean_name(name)
            self._values[name] = value
            self._times_last_updated[name] = time.time()

            # Check to see if any ORed triggers have been met.
            for tid, criteria in self._or_trigger_callables.items():
                # if verbose: print('checking tid:', tid, criteria)

                # Skip this trigger if it does not use the current name.
                if name not in criteria:
                    # if verbose: print('name not in criteria')
                    continue

                # Skip this trigger if it's already been activated.
                # if tid in self._or_trigger_results:
                if self.has_triggered(tid):
                    # if verbose: print('tid already in results')
                    continue

                # Check to see if this trigger has any dependencies blocking it.
                has_untriggered_dep = False
                for dep_tid in self._or_trigger_dependencies.get(tid, []):
                    # if verbose: print('checking dep_tid %s triggered: %s' % (dep_tid, self.has_triggered(dep_tid)))
                    if not self.has_triggered(dep_tid):
                        has_untriggered_dep = True
                        break
                if has_untriggered_dep:
                    # if verbose: print('skipping because this has an untriggered dependency')
                    continue

                # Otherwise, check the value against the trigger's logic.
                target_value = criteria[name]
                if isinstance(target_value, bool) and target_value == bool(value):
                    # If target is a boolean, then check boolean value of incoming value.
                    # if verbose: print('triggered based on boolean!')
                    self._or_trigger_results[tid] = True
                elif target_value == value:
                    # Otherwise check literal value.
                    # if verbose: print('triggered based on literal!')
                    self._or_trigger_results[tid] = True
                # else:
                    # if verbose: print('not triggered')

    def check_or_trigger(self, tid):
        """
        Returns True if the trigger has been activated.
        Note, to avoid a memory leak, once the activated, the trigger will be deleted, so further checks will fail.
        """
        assert tid in self._or_trigger_callables
        if tid in self._or_trigger_results:
            del self._or_trigger_callables[tid]
            return True
        return False

    def set_or_trigger(self, **kwargs):
        """
        Registers a check for one or more state values going from a high to low state or vice versa.

        As an ORed trigger, the criteria will be met if any one of the sub-conditions are met.

        The optional parameter `dependencies` is a list of TriggerHandle instances representing other triggers that must be activated before
        this new trigger is allowed to activate.

        Returns a helper callable that will return True when the trigger has been activated.
        """
        dependencies = kwargs.pop('dependencies', None) or []
        assert isinstance(dependencies, (tuple, list))
        tid = uuid.uuid4()
        for k, v in kwargs.items():
            self.clean_name(k)
        self._or_trigger_callables[tid] = dict(kwargs)
        for dep in dependencies:
            self._or_trigger_dependencies.setdefault(tid, [])
            self._or_trigger_dependencies[tid].append(dep.tid)
        #return partial(self.check_or_trigger, tid=tid)
        return TriggerHandle(tid=tid, callback=self.check_or_trigger, state=self)

    def unset_trigger(self, tid):
        if tid in self._or_trigger_dependencies:
            del self._or_trigger_dependencies[tid]
        if tid in self._or_trigger_callables:
            del self._or_trigger_callables[tid]
        if tid in self._or_trigger_results:
            del self._or_trigger_results[tid]


class QRMatch(object):
    """
    Wraps a QR match message and binds manipulation functionality.
    """

    def __init__(self, match, euler_z):
        self.match = match # the raw match message
        self.euler_z = euler_z
        self.t0 = time.time()

    @property
    def position_ratio(self):
        """
        Returns the ratio of how far the match along the x-axis.
        Bounded in [0:1].
        A value < 0.4 means QR code is to the left.
        A value > 0.6 means QR code is to the right.
        """
        match = self.match
        if not match:
            return
        x = (match.a[0] + match.b[0] + match.c[0] + match.d[0])/4.
        width = match.width
        # Ideally, this should be 0.5, indicating the QR code is exactly in the center of the horizontal view.
        # In practice, we'll likely never accomplish this, but we'll try to get it within [0.4:0.6]
        position_ratio = x/float(width)
        return position_ratio


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

    def test_trigger(self):
        s = State()
        s.register('encoder_a')
        s.register('encoder_b')
        s.set('encoder_a', 0)
        s.set('encoder_b', 0)

        trigger_up = s.set_or_trigger(encoder_a=True, encoder_b=True)
        trigger_down = s.set_or_trigger(encoder_a=False, encoder_b=False, dependencies=[trigger_up])
        self.assertEqual(trigger_up(), False)
        self.assertEqual(trigger_up(), False)
        self.assertEqual(trigger_down(), False)
        self.assertEqual(trigger_down(), False)
        s.set('encoder_a', 0)
        s.set('encoder_b', 0)

        # Because the down trigger is dependent on the up trigger, the up trigger must activate first.
        # Therefore, no matter how many times our target values match our activation criteria, our trigger will remain unactivated.
        self.assertEqual(trigger_down(), False)
        s.set('encoder_a', -45)
        self.assertEqual(trigger_up(), True)
        with self.assertRaises(AssertionError):
            trigger_up()
        s.set('encoder_b', -45)

        self.assertEqual(trigger_down(), False)
        self.assertEqual(trigger_down(), False)
        s.set('encoder_a', 0)
        s.set('encoder_b', 0)
        # Now that the up trigger has activated, the down trigger is allowed to activate.
        self.assertEqual(trigger_down(), True)
        with self.assertRaises(AssertionError):
            trigger_down()

if __name__ == '__main__':
    unittest.main()
