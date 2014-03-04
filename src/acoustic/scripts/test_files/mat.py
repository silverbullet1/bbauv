import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import random
final_data = []
fig1 = plt.figure()
ax = plt.axes(xlim=(0,360), ylim=(0,5))
l, = ax.plot([], [], lw=2)

def init():
    l.set_data([],[])
    return l,

def anime(i):
    x_data = np.linspace(0,2,1000)
    y_data = np.sin(2*np.pi*(x_data - 0.01*i))
    l.set_data(x_data, y_data)
    return l
anim = animation.FuncAnimation(fig1, anime, init_func=init, frames=200, interval=20, blit=True)
plt.show()
