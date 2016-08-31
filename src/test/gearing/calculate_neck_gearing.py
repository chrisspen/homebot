"""
2016.8.10 CKS
Calculate the theoretical maximum number of counts the pan motor should detect
during a full 360 degree rotation.
"""
from math import *

big_od = 105
small_od = 21
gear_overlap = 2.5
motor_gear_ratio = 51.45/1. # 51.45 revs of motor axle turns gear 1 rev
encoder_ratio =  6#12 # 1 rev of motor axle causes 12 counts

big_circumference = pi * (big_od - gear_overlap)
# print 'big_circumference:', big_circumference

small_circumference = pi * (small_od - gear_overlap)
# print 'small_circumference:', small_circumference

total_small_gear_revs = big_circumference/small_circumference
# print 'total_small_gear_revs:', total_small_gear_revs

total_motor_revs = total_small_gear_revs * motor_gear_ratio
# print 'total_motor_revs:', total_motor_revs

total_encoder_counts = total_motor_revs * encoder_ratio
print 'total_encoder_counts:', total_encoder_counts
