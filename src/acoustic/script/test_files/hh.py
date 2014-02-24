import matplotlib.pyplot as plt
import random

x = [i for i in range(360)]
y = [int(random.random()*5) for i in range(360)]

plt.plot(x, y)
plt.savefig("pmusic.png")
plt.show()
