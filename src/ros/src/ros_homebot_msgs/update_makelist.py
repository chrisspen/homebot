#!/usr/bin/env python
"""
Updates CMakeList.txt to include all .msg and .srv files.
"""
import os, re

msgs = sorted([fn for fn in os.listdir('msg') if fn.endswith('.msg')])
print 'messages:', msgs

srvs = sorted([fn for fn in os.listdir('srv') if fn.endswith('.srv')])
print 'services:', srvs

content = open('CMakeLists.txt', 'r').read()

MESSAGE_PATTERN = re.compile(r'add_message_files\(FILES.*?\)', flags=re.I|re.M|re.DOTALL)
#print MESSAGE_PATTERN.findall(open('CMakeLists.txt', 'r').read())
repl = 'add_message_files(FILES\n    %s\n)' % ('\n    '.join(msgs),)
content = MESSAGE_PATTERN.sub(repl, content)
print content

SERVICE_PATTERN = re.compile(r'add_service_files\(FILES.*?\)', flags=re.I|re.M|re.DOTALL)
repl = 'add_service_files(FILES\n    %s\n)' % ('\n    '.join(srvs),)
content = SERVICE_PATTERN.sub(repl, content)
print content

open('CMakeLists.txt', 'w').write(content)
