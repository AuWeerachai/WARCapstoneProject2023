import numpy as np

start = np.array([6,7,8,9,0])
home = np.array([1,2,3,4,5])
both = np.transpose(np.vstack((start,home)))
print(both)