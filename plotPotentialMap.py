import pandas as pd
import numpy as np
from matplotlib import pyplot as pl
from mpl_toolkits.mplot3d import Axes3D


potentialMap = pd.read_csv("./potentialMap.csv", header=None)
potentialMap.columns = ["x", "y", "potential"]
print(potentialMap)
potentialMap = potentialMap.pivot(index='x', columns='y', values='potential')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("x", size = 16, color = "b")
ax.set_ylabel("y", size = 16, color = "b")
ax.set_zlabel("potential", size = 16, color = "b")
ax.plot_surface(potentialMap.index, potentialMap.columns, potentialMap.values, cmap = "plasma_r")
pl.show()
