# Full Kalman filter for 1 dimension.

def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / float(var1 + var2)
    new_var = 1/(1./var1 + 1./var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0
sig = 10000
sig = .000000001

#Please print out ONLY the final values of the mean
#and the variance in a list [mu, sig]. 

#for measurement, motion in zip(measurements, motion):
#    mu, sig = update(mu, sig, measurement, measurement_sig)
#    print [mu, sig]
#    mu, sig = predict(mu, sig, motion, motion_sig)
#    print [mu, sig]

print update(1, 1, 3, 1)
