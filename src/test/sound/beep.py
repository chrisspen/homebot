"""
sudo apt-get install python-pyaudio

sudo apt-get install libportaudio-dev
pip install pyaudio
"""
import time
import math
import pyaudio
import wave

class Beeper(object):

    def __init__(self, **kwargs):
        self.bitrate = kwargs.pop('bitrate', 16000)
        self.channels = kwargs.pop('channels', 1)
        self._p = pyaudio.PyAudio()
#         self.format = pyaudio.paInt16
        self.format = self._p.get_format_from_width(1)
        self.stream = self._p.open(
            format = self.format,
            channels = self.channels,
            rate = self.bitrate,
            output = True,
        )
        self._queue = []

        self._last_generator = None
        self.frame_overlaps = kwargs.pop('frame_overlaps', 1000)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stream.stop_stream()
        self.stream.close()

    def tone(self, frequency, length=1000, play=False, **kwargs):
        """
        playTone(rate=88000,wave=400,time=1,channel=2)
        """

        number_of_frames = int(self.bitrate * length/1000.)

        record = False
        x = 0
        y = 0
        while 1:
            x += 1
            v = math.sin(x/((self.bitrate/float(frequency))/math.pi))

            # Find where the sin tip starts.
            if round(v, 3) == +1:
                record = True

            if record:
                self._queue.append(chr(int(v*127+128)))
                y += 1
                if y > number_of_frames and round(v, 3) == +1:
                    # Always end on the high tip of the sin wave to clips align.
                    break

    def play(self):
#         for chunk in self._queue:
#             self.stream.write(chunk)
        #sound = reduce(lambda a, b: a+b, self._queue, '')
        sound = ''.join(self._queue)
        self.stream.write(sound)
        time.sleep(0.1)
        self._queue = []

    def save(self, fn):
        waveFile = wave.open(fn, 'wb')
        waveFile.setnchannels(self.channels)
        waveFile.setsampwidth(self._p.get_sample_size(self.format))
        waveFile.setframerate(self.bitrate)
        waveFile.writeframes(b''.join(self._queue))
        waveFile.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('-f', dest='frequency', type=int, default=750,
        help='beep at N Hz, where 0 < N < 20000.  As a general ballpark, the regular terminal beep is around 750Hz.  N is not, incidentally, restricted to whole numbers.')
    parser.add_argument('-l', dest='length', type=int, default=1000,
        help='beep for N milliseconds')
    parser.add_argument('-r', dest='repetitions', type=int, default=1,
        help='specify the number of repetitions (defaults to 1)')

    args = parser.parse_args()
    #print(args.accumulate(args.integers))
    with Beeper(**args.__dict__) as beeper:
        beeper.beep(**args.__dict__)
