

def calculate_discharge_time_abs(t0, v0, t1, v1, dead_level=0.8):
    #y = mx + b => v = m*t + b => (v - b)/m = t
    #x = t
    #y = v
    #m = dy/dx
    #assumes v0 = voltage at 100%
    m = (v1 - v0)/float(t1 - t0)
    print 'm:', m
    b = v0 - m*t0
    print 'b:', b
    
    v_dead = v0 * dead_level
    print 'v_dead:', v_dead

    t_dead = (v_dead - b)/m
    #t_dead = ((v0 * 0.8) - b)/m
    return t_dead
 
#battery 1   
# t_dead_abs = calculate_discharge_time_abs(
#     t0=0,#11:10
#     v0=7.56,
#     t1=60+30+7,#minutes, 12:47
#     v1=3.86,
# )

#battery 2
# t_dead_abs = calculate_discharge_time_abs(
#     t0=0,#2pm
#     v0=7.67,
#     t1=60+36,#3:36
#     v1=3.85,
# )

#12v battery
t_dead_abs = calculate_discharge_time_abs(
    t0=0,
    v0=11.84,
    t1=72,
    v1=10.84,
)

print 't_dead_abs (min):', t_dead_abs
print 't_dead_abs (hrs):', t_dead_abs*(1/60.)
