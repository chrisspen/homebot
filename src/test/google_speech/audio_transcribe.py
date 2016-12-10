#!/usr/bin/env python
"""
https://github.com/Uberi/speech_recognition/blob/master/examples/audio_transcribe.py

Works poorly. Accuracy rarely accurate.
"""
from os import path
import speech_recognition as sr

tests = [
    #("testing one two three.flac", 'testing 1 2 3 testing 1 2 3'),
    ('testing one two three a b c d e f g.flac', 'testing one two three a b c d e f g'),
]
results = []

for fn, expected_text in tests:
    
    AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), fn)
    
    # use the audio file as the audio source
    r = sr.Recognizer()
    with sr.AudioFile(AUDIO_FILE) as source:
        audio = r.record(source) # read the entire audio file
    
    # recognize speech using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        predicted_text = r.recognize_google(audio)
        print repr(predicted_text)
        print repr(expected_text)
        results.append(predicted_text == expected_text)
        print("Google Speech Recognition thinks you said " + predicted_text)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

print '='*80
print 'results:', results
