#!/usr/bin/env python
"""
http://www.confusedcoders.com/random/speech-recognition-in-python-with-cmu-pocketsphinx

sudo apt-get install python-pocketsphinx
sudo apt-get install pocketsphinx-hmm-wsj1
sudo apt-get install pocketsphinx-lm-wsj
"""
 
import os, sys
import pocketsphinx, wave
 
if __name__ == "__main__":
 
    #hmdir = "/usr/share/pocketsphinx/model/hmm/wsj1"
    hmdir = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k/"
    assert os.path.isdir(hmdir)
    #lmdir = "/usr/share/pocketsphinx/model/lm/wsj/wlist5o.3e-7.vp.tg.lm.DMP"
    lmdir = "/usr/share/pocketsphinx/model/lm/en_US/hub4.5000.DMP"
    assert os.path.exists(lmdir)
    #dictd = "/usr/share/pocketsphinx/model/lm/wsj/wlist5o.dic"
    dictd = "/usr/share/pocketsphinx/model/lm/en_US/cmu07a.dic"
    assert os.path.isfile(dictd)
    
    wavfile = sys.argv[1]
 
    #speechRec = pocketsphinx.Decoder(hmm = hmdir, lm = lmdir, dict = dictd)
    
#     MODELDIR = '/usr/share/pocketsphinx/model'
#     DATADIR = "../../../test/data"
     
    # Create a decoder with certain model
    config = pocketsphinx.Decoder.default_config()
    config.set_string('-hmm', hmdir)
    config.set_string('-lm', lmdir)
    config.set_string('-dict', dictd)
    
    # Decode streaming data.
    speechRec = pocketsphinx.Decoder(config)
    
    #wavFile = file(wavfile,'rb')
    #speechRec.decode_raw(wavFile)
    
    speechRec.start_utt()
    #stream = open(path.join(DATADIR, 'goforward.raw'), 'rb')
    wf = wave.open(wavfile, 'rb')
    wave_data = wf.readframes(1024)
    for frame in wave_data:
        speechRec.process_raw(frame, False, False)
    speechRec.end_utt()
    
    #result = speechRec.get_hyp()
    hypothesis = speechRec.hyp()
 
    print 'hypothesis:', hypothesis
    