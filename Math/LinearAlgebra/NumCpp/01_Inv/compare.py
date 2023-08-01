import numpy as np

with open('conv_inv_py.dat', 'rb') as f:
    py = np.fromfile(f, dtype=np.float64)
with open('conv_inv_cpp.dat', 'rb') as f:
    cpp = np.fromfile(f, dtype=np.float64)

tolerance = 0.00001
total = 0
count = 0
for i in range(100 * 100):
    diff = abs(py[i] - cpp[i])
    if diff > tolerance:
        print(f'{py[i]} = {cpp[i]}')
        count = count + 1
    total = total + diff

print(f'tolerance: {tolerance}')
print(f'    total: {total}')
print(f'    count: {count}')