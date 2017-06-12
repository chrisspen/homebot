"""
2015.12.18 CKS
Test of a simple proportional speed controller.
"""

def get_speed(actual_pos, expected_pos):
    #TODO:handle wrapping (e.g. actual_pos=1, expected_pos=358, should turn CCW, not CW)
    max_angle = 360
    max_speed = 255
    direction = +1
    pos_diff = expected_pos - actual_pos # [-360:360]
    if pos_diff > max_angle/2:
        direction = -1
        pos_diff = max_angle - pos_diff
    speed = (pos_diff + max_angle/2)/(max_angle/2*2.)*(max_speed*2.) - max_speed
    speed *= direction
    return speed

inputs = (
    #(actual_pos, expected_pos, expected_speed),
    (0, 359, -1),
    (0, 340, -28),
    (0, 260, -142),
    (0, 181, -254),
    (0, 179, 254),
    (0, 15, 21),
    (0, 45, 64),
    (0, 90, 128),
    (0, 170, 241),
)

for actual_pos, expected_pos, expected_speed in inputs:
    speed = get_speed(actual_pos, expected_pos)
    speed = round(speed, 0)
    print 'pos: %s -> %s, speed.expected %s, speed.actual: %s' % (expected_pos, actual_pos, expected_speed, speed)
    assert speed == expected_speed, 'Expected %s but calculated %s.' % (expected_speed, speed)
