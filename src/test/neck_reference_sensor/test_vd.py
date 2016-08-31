#/usr/bin/env python

Vout = lambda Vin, R1, R2: Vin * R2/float(R1 + R2)

pullup_r = 20000

pulldown_r_low = 9000
#pulldown_r_high = 20000 #theoretical
pulldown_r_high = 12000

pulldown_fixed = 0 # too little, both states low
pulldown_fixed = 20000 # both states still low
pulldown_fixed = 30000 # too much, both states high

vout_min = Vout(Vin=4.8, R1=pullup_r, R2=pulldown_r_low+pulldown_fixed)
vout_max = Vout(Vin=4.8, R1=pullup_r, R2=pulldown_r_high+pulldown_fixed)

print 'vout min,max: %.02f %.02f' % (vout_min, vout_max)