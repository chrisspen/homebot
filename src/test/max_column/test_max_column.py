#!../../../.env/bin/python
import os
import numpy as np
import time

a = np.array([
    [1,0,3],
    [0,2,1],
    [0.1,0,0],
])

print a
row = 1
col = 2
print a[row][col]
assert a[row][col] == 1

expected_max_rows = [0, 1, 0]
expected_max_values = [1, 2, 3]
print 'expected_max_rows:', expected_max_rows
print 'expected_max_values:', expected_max_values

t0 = time.time()
actual_max_rows = list(np.argmax(a, axis=0))
td = time.time() - t0
actual_max_values = list(np.amax(a, axis=0))
print 'td:', round(td, 4)
print 'actual_max_rows:', actual_max_rows
print 'actual_max_values:', actual_max_values
assert actual_max_rows == expected_max_rows
assert actual_max_values == expected_max_values
