#!/usr/bin/env python
"""
2016.12.10 CKS

Loops indefinitely, recording from the microphone, sending short segments of speech to Google for recognition.

Seems to work ok.

I tested this on a laptop and I found the amplification setting on my built-in microphone drastically effected results.
At first, it was set to 100% amplification, which picked up a huge amount of ambient noise, causing a 0% recognition rate.
The adjust_for_ambient_noise() function doesn't seem work if there's too much noise.
When I reduced the amplification to about 30%, so that my voice was the only thing causing the sound bar to jump, recognition jumped to around 90%.
"""
import sys
import time
from StringIO import StringIO

import speech_recognition as sr

class HideOutput():

    def __init__(self, stdout=True, stderr=True):
        self.hide_stdout = stdout
        self.hide_stderr = stderr
        self._stdout = None
        self._stderr = None

    def __enter__(self):
        if self.hide_stdout:
            self._stdout = sys.stdout
            sys.stdout = StringIO()
        if self.hide_stderr:
            self._stderr = sys.stderr
            sys.stderr = StringIO()

    def __exit__(self, *args):
        if self.hide_stdout:
            sys.stdout = self._stdout
        if self.hide_stderr:
            sys.stderr = self._stderr

# You may need to change this to suit your hardware. Try None.
mic_index = 4

r = sr.Recognizer()
#print 'a'
m = sr.Microphone(device_index=mic_index)
#print 'b'

#set threhold level
try:
    with m as source:
        r.adjust_for_ambient_noise(source)
    print("Set minimum energy threshold to {}".format(r.energy_threshold))
except IOError as e:
    print e
    print 'Try different microphone device index:', sr.Microphone.list_microphone_names()
    sys.exit(1)

do_prompt = True
while 1:

    try:

        if do_prompt:
            print("Listening!")
            do_prompt = False

        with sr.Microphone(device_index=mic_index) as source:
            audio = r.listen(source, timeout=0.1)
            do_prompt = True

        t0 = time.time()
        print 'Recognizing...'
        text = r.recognize_google(audio)
        td = time.time() - t0
        print 'Response seconds:', td

        print(text)
    except sr.UnknownValueError:
        # We heard you, but Google couldn't figure out what you said.
        pass
    except sr.WaitTimeoutError:
        pass
