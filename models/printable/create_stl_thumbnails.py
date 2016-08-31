#!/usr/bin/env python
"""
Creates PNG thumbnails for all STL files.
"""
from __future__ import print_function
import os, sys, tempfile, argparse, time, csv

PRINTING_GUIDE_FN = 'printing_recommendations.csv'
PRINTING_GUIDE_HEADERS = ['filename', 'wall_thickness', 'color', 'infill_percent', 'platform_adhesion', 'support_type', 'obsolete']

def generate_thumbnails(force=False, **kwargs):
    
    unique_stl_filenames = set()
    
    for fn in os.listdir('.'):
        if not fn.endswith('.stl'):
            continue
        
        unique_stl_filenames.add(fn)
        #print(unique_stl_filenames)
        base_fn, ext = os.path.splitext(fn)
        thumbnail_fn = base_fn + '.png'
        
        # Track if STL has been modified since last thumbnail was generated.
        ts_stale = False
        if os.path.isfile(thumbnail_fn):
            stl_ts = os.path.getmtime(fn)
            thumbnail_ts = os.path.getmtime(thumbnail_fn)
            ts_stale = thumbnail_ts < stl_ts
        
        # Skip if thumbnail exists and it's up to date.
        if not force and os.path.isfile(thumbnail_fn) and not ts_stale:
            continue
        
        tmp_fh, tmp_fn = tempfile.mkstemp()
        tmp_fout = os.fdopen(tmp_fh, 'w')
        tmp_fout.write('import("%s");' % os.path.abspath(fn))
        tmp_fout.close()
        
        cmd = 'openscad --autocenter --projection=ortho --render --viewall -o %s %s' % (os.path.abspath(thumbnail_fn), tmp_fn)
        print(cmd)
        os.system(cmd)
        
    data = {}
    if os.path.isfile(PRINTING_GUIDE_FN):
        for row in csv.DictReader(open(PRINTING_GUIDE_FN)):
            if not row['filename'].strip():
                continue
            data[row['filename']] = row
    
    writer = csv.DictWriter(open(PRINTING_GUIDE_FN, 'w'), fieldnames=PRINTING_GUIDE_HEADERS)
    writer.writerow(dict(zip(PRINTING_GUIDE_HEADERS, PRINTING_GUIDE_HEADERS)))
    filenames = sorted(set(list(data) + list(unique_stl_filenames)))
    for filename in filenames:
        row = data.get(filename, {})
        row['filename'] = filename
        row['obsolete'] = 0
        if filename not in unique_stl_filenames:
            row['obsolete'] = 1
        writer.writerow(row)
        
if __name__ == '__main__':
        
    parser = argparse.ArgumentParser(description='Creates PNG thumbnails for all STL files.')
    parser.add_argument('--force', dest='force', action='store_true',
        help='If given, re-generates a thumbnail even if one already exists.')
    
    args = parser.parse_args()
    
    generate_thumbnails(**args.__dict__)
