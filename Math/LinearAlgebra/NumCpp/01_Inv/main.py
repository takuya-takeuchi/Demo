import numpy as np

with open('conv.dat', 'rb') as f:
    conv = np.fromfile(f, dtype=np.float32).reshape(100, 100)

conv_inv = np.linalg.inv(conv.astype(np.float64))

with open(f'conv_inv_py.dat', 'wb') as f:
    f.write(conv_inv)