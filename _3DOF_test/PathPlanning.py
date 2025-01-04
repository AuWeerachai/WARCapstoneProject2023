import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt



dz = np.array([-0.5,0,1/64,1/16,0.125,0.25,0.375,0.5,0.625,0.75,0.875,0.9375,31/32,63/64,1])
w = np.array([-1.2,1,1.005,1.0203113,1.04155,1.087655304,1.139415,1.2,1.273723,1.369762709,1.513176941,1.63098,1.724772,1.79743,2])
print(np.shape(dz))
print(np.shape(w))
guess = np.polyfit(dz,w,30)
p = np.poly1d(guess)


Cmax = 20
z1 = 5
A = (Cmax - z1)
z2 = 15
rdz = z2 - z1
rat = rdz/A
x1 = 7
x2 = 15
k = np.pi/(p(rat)*(x2-x1))
# k = np.pi/(-1.2*(x2-x1))
x = np.linspace(x1,x2,10)
z = A*np.sin((x - x1)*k) +z1
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(x, z, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
ax.scatter(x, z, color='green', marker='o', s=40)
ax.scatter(x1, z1, color='black', marker='o', s=40)
ax.scatter(x2, z2, color='black', marker='o', s=40)
ax.set_xlabel('x')
ax.set_ylabel('z')
plt.show()
print(z2 - (A*np.sin((x2 - x1)*k) +z1))


xp = np.linspace(0, 1, 80)
plt.plot(xp, p(xp), '-')
ax.scatter(dz, w, color='green', marker='o', s=40)
ax.set_xlabel('dz')
ax.set_ylabel('w')
plt.show()