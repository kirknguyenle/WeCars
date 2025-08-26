import numpy as np
import matplotlib.pyplot as plt
import feature_generation as fg

inc = 1

path = [[0,0],[0,0]]

xf = path[0][len(path[0])-1]
fg.generateLine(path, 25, inc)
fg.linearFixedLaneChange(path, 10, 3.5, inc)
fg.generateLine(path, 10, inc)
fg.linearFixedLaneChange(path, 10, -3.5, inc)
fg.generateLine(path, 100, inc)


plt.plot(path[0], path[1], label = "path")
plt.show()