from pylab import *
import time
import numpy as np
import random

ion()
tstart = time.time()               # for profiling
x = arange(0,360,1)            # x-array
line, = plot(x,sin(x))
for i in [int(5*random.random()) for i in range(360)]:
    line.set_ydata(i)  # update the data
    draw()                         # redraw the canvas

print 'FPS:' , 50/(time.time()-tstart)
