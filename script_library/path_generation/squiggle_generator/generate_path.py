import numpy as np
import matplotlib.pyplot as plt
import feature_generation as fg

inc = 1

path = [[0,0],[0,0]]

xf = path[0][len(path[0])-1]
fg.generateLine(path, 500, inc)
fg.jumpLaneChange(path, 2, inc)
fg.generateLine(path, 500, inc)
fg.linearLaneChange(path, 0.02, -1, 1)
fg.generateLine(path, 500, inc)
fg.sinLaneChange(path, 500, 1, inc)

plt.plot(path[0], path[1], label = "path")
plt.show()