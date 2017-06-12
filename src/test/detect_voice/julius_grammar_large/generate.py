#!/usr/bin/env python
"""
Afterwards run:

    time mkdfa sample

"""
import os, sys

fin = open('../beep/beep-1.0', 'r')

grammar = open('sample.grammar', 'w')
grammar.write("""
S : NS_B WORD_LOOP NS_E
WORD_LOOP : WORD_LOOP WORD
WORD_LOOP : WORD
""")

voca = open('sample.voca', 'w')
voca.write("""
% NS_B
<s>        sil

% NS_E
</s>        sil

% WORD
""")

lines = fin.readlines()
total = len(lines)
i = 0
for line in lines:
    i += 1
    sys.stdout.write('\r%i of %i (%.02f%%)' % (i, total, i/float(total)*100))
    sys.stdout.flush()
    line = line.strip()
    if line.startswith('#') or line.startswith(r'%'):
        continue
    work = line.split(' ')[0]
    voca.write(line+'\n')
