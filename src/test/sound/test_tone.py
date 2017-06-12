from math import *
from beep import Beeper

#br = 88000
br = 16000

beeper = Beeper(bitrate=br, channels=2)

def happy(reps=1):
    length = 0.05*1000
    for rep in xrange(reps):
        for f in xrange(700, 1000+1, 100):
            print f
            beeper.tone(frequency=f, length=length)
    beeper.play()

def chirtle():
    happy(3)

def real_happy(reps=1):
    length = 0.1*1000
    for rep in xrange(reps):
        for f in xrange(200, 1000+1, 100):
            print f
            beeper.tone(frequency=f, length=length)
        beeper.play()

def sad():
    length = 0.05*1000
    for f in xrange(1000, 700-1, -100):
        print f
        beeper.tone(frequency=f, length=length)
    beeper.play()

def real_sad(reps=1):
    length = 0.1*1000
    for rep in xrange(reps):
        for f in xrange(1000, 200-1, -100):
            print f
            beeper.tone(frequency=f, length=length)
    beeper.play()

def whistle():
    length = 0.1*1000
    for f in xrange(700, 1000+1, 100):
        print f
        beeper.tone(frequency=f, length=length)
    beeper.tone(frequency=700, length=.5*1000)
    beeper.play()

def alarmed():
    # up and down
    for i in xrange(5):
        happy()
        sad()

def whine():
    #base_length = 0.05*1000
    i = 0
    for f in xrange(1000, 800-1, int(round(-25/2.))):
        i += 1
        length = log(i+1) * 250/2./2.
        print f, length
        beeper.tone(frequency=f, length=length)
    beeper.play()
    #beeper.save('whine.wav')

print 'happy'
happy()

print 'chirtle'
chirtle()

print 'real happy'
real_happy()

print 'sad'
sad()

print 'real sad'
real_sad()

print 'whistle'
whistle()

print 'alarmed'
alarmed()

print 'whine'
whine()

