#!../../../.env/bin/python

import pyaudio
import wave
import audioop
from collections import deque
import os
import urllib2
import urllib
import time
import math

LANG_CODE = 'en-US'  # Language to use

#GOOGLE_SPEECH_URL = 'https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&pfilter=2&lang=%s&maxresults=6' % (LANG_CODE)
GOOGLE_SPEECH_URL = 'https://www.google.com/speech-api/v1/recognize?client=chromium&lang={lang}&maxresults=10'.format(lang=LANG_CODE)

FLAC_CONV = 'flac -f'  # We need a WAV to FLAC converter. flac is available
                       # on Linux

# Microphone stream config.
CHUNK = 1024  # CHUNKS of bytes to read each time from mic
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
THRESHOLD = 2500  # The threshold intensity that defines silence
                  # and noise signal (an int. lower than THRESHOLD is silence).

SILENCE_LIMIT = 1  # Silence limit in seconds. The max ammount of seconds where
                   # only silence is recorded. When this time passes the
                   # recording finishes and the file is delivered.

PREV_AUDIO = 0.5  # Previous audio (in seconds) to prepend. When noise
                  # is detected, how much of previously recorded audio is
                  # prepended. This helps to prevent chopping the beggining
                  # of the phrase.


def audio_int(num_samples=50):
    """ Gets average audio intensity of your mic sound. You can use it to get
        average intensities while you're talking and/or silent. The average
        is the avg of the 20% largest intensities recorded.
    """

    print "Getting intensity values from mic."
    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    values = [math.sqrt(abs(audioop.avg(stream.read(CHUNK), 4))) 
              for x in range(num_samples)] 
    values = sorted(values, reverse=True)
    r = sum(values[:int(num_samples * 0.2)]) / int(num_samples * 0.2)
    print " Finished "
    print " Average audio intensity is ", r
    stream.close()
    p.terminate()
    return r

def list_input_devices():
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        data = p.get_device_info_by_index(i)
        if data.get('maxInputChannels') > 0:
            print i, data
        else:
            print i, None

def get_input_device_index(name):
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        data = p.get_device_info_by_index(i)
        if data.get('maxInputChannels') > 0 and name in data['name']:
            return i

def listen_for_speech(threshold=THRESHOLD, num_phrases=-1, input_device='default', input_file=None, stt=True):
    """
    Listens to Microphone, extracts phrases from it and sends it to 
    Google's TTS service and returns response. a "phrase" is sound 
    surrounded by silence (according to threshold). num_phrases controls
    how many phrases to process before finishing the listening process 
    (-1 for infinite). 
    """
    
    input_device_index = get_input_device_index(name=input_device)

    #Open stream
    p = pyaudio.PyAudio()

    stream = None
    if input_file:
        #http://stackoverflow.com/questions/6951046/pyaudio-help-play-a-file
        print 'Opening file %s...' % input_file
        wf = wave.open(input_file, 'rb')
#         stream = p.open(
#             format=p.get_format_from_width(wf.getsampwidth()),
#             channels = wf.getnchannels(),
#             rate = wf.getframerate(),
#             output = True)
        wave_data = wf.readframes(CHUNK)
#         print 'wave_data:', len(wave_data)
#         for i, f in enumerate(wave_data):
#             print i, repr(f), type(f)

    else:
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            input_device_index=input_device_index,
            frames_per_buffer=CHUNK)

    def get_frame():
        if input_file:
            for frame in wave_data:
                yield frame
        else:
            while 1:
                frame = stream.read(CHUNK)
                yield frame

    print "* Listening mic. "
    audio2send = []
    cur_data = ''  # current chunk  of audio data
    rel = RATE/CHUNK
    slid_win = deque(maxlen=SILENCE_LIMIT * rel)
    #Prepend audio from 0.5 seconds before noise was detected
    prev_audio = deque(maxlen=PREV_AUDIO * rel) 
    started = False
    n = num_phrases
    response = []

    frame_count = 0
    get_frame_iter = get_frame()
    while 1:#(num_phrases == -1 or n > 0):
        frame_count += 1
        print 'frame:', frame_count
        #cur_data = stream.read(CHUNK)
        try:
            cur_data = get_frame_iter.next()
        except StopIteration:
            break
#         print 'cur_data:', cur_data, repr(cur_data), len(cur_data)
        #slid_win.append(math.sqrt(abs(audioop.avg(cur_data, 4))))
        slid_win.append(cur_data)
        #print slid_win[-1]
        voice = sum([x > THRESHOLD for x in slid_win])
        print 'threshold:', voice, THRESHOLD
        if voice > 0:
            if(not started):
                print "Starting record of phrase"
                started = True
            audio2send.append(cur_data)
        elif (started is True):
            print "Finished"
            # The limit was reached, finish capture and deliver.
            filename = save_speech(list(prev_audio) + audio2send, p)
            # Send file to Google and get response
            if stt:
                r = stt_google_wav(filename) 
                if num_phrases == -1:
                    print "Response", r
                else:
                    response.append(r)
                # Remove temp file. Comment line to review.
                os.remove(filename)
            else:
                print 'Skipping STT.'
            # Reset all
            started = False
            slid_win = deque(maxlen=SILENCE_LIMIT * rel)
            prev_audio = deque(maxlen=0.5 * rel) 
            audio2send = []
            n -= 1
            print "Listening ..."
        else:
            prev_audio.append(cur_data)

    print "* Done recording"
    if stream is not None:
        stream.close()
    p.terminate()

    print 'response:', response
    return response


def save_speech(data, p):
    """ Saves mic data to temporary WAV file. Returns filename of saved 
        file """

    filename = 'output_'+str(int(time.time()))
    # writes data to WAV file
    data = ''.join(data)
    wf = wave.open(filename + '.wav', 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setframerate(16000)  # TODO make this value a function parameter?
    wf.writeframes(data)
    wf.close()
    return filename + '.wav'


#https://github.com/gillesdemey/google-speech-v2
#Note, only 50 requests per day
def stt_google_wav(audio_fname):
    """ Sends audio file (audio_fname) to Google's text to speech 
        service and returns service's response. We need a FLAC 
        converter if audio is not FLAC (check FLAC_CONV). """

    print "Sending ", audio_fname
    #Convert to flac first
    filename = audio_fname
    del_flac = False
    if 'flac' not in filename:
        del_flac = True
        print "Converting to flac"
        print FLAC_CONV + filename
        os.system(FLAC_CONV + ' ' + filename)
        filename = filename.split('.')[0] + '.flac'

    f = open(filename, 'rb')
    flac_cont = f.read()
    f.close()

    # Headers. A common Chromium (Linux) User-Agent
    #https://gist.github.com/alotaiba/1730160
    #audio/x-flac; rate=16000
    hrs = {"User-Agent": "Mozilla/5.0 (X11; Linux i686) AppleWebKit/535.7 (KHTML, like Gecko) Chrome/16.0.912.63 Safari/535.7", 
           'Content-type': 'audio/x-flac; rate=16000'}  

    req = urllib2.Request(GOOGLE_SPEECH_URL, data=flac_cont, headers=hrs)
    print "Sending request to Google TTS"
    #print "response", response
    try:
        p = urllib2.urlopen(req)
        response = p.read()
        print 'google sst response:', response
        res = eval(response)['hypotheses']
    except Exception as e:
        print "Couldn't parse service response: %s" % e
        if 'not found' in str(e):
            raise
        res = None

    if del_flac:
        os.remove(filename)  # Remove temp file

    return res


if(__name__ == '__main__'):
    #listen_for_speech(input_device='HDA Intel PCH: ALC269VB Analog')  # listen to built-in mic.
    
    #TODO:read mp3 directly? https://sourceforge.net/p/lame/mailman/message/11484009/
    listen_for_speech(input_file='sorry_dave.wav')
    #print stt_google_wav('hello.flac')  # translate audio file
    #audio_int()  # To measure your mic levels
