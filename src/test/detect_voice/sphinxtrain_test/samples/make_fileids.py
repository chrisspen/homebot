#!/usr/bin/env python
import os, sys

def touch(path):
    with open(path, 'a'):
        os.utime(path, None)

fout = open('sample.fileids', 'wb')
fout2 = open('sample.transcription', 'wb')
for fn in sorted(os.listdir('.')):
    if not fn.endswith('.wav'):
        continue
    base_fn = os.path.splitext(fn)[0]
    txt_fn = base_fn + '.txt'
    touch(txt_fn)
    text = open(txt_fn).read().strip()
    if text and not text.startswith('#'):
        fout.write('samples/%s\n' % base_fn)
        fout2.write('<s> %s <s> (%s)\n' % (text, base_fn))
print 'Done.'
