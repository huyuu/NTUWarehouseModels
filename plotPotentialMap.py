import pandas as pd
import numpy as np
from matplotlib import pyplot as pl
from mpl_toolkits.mplot3d import Axes3D


potentialMap = pd.read_csv("./potentialMap.csv", header=None)
potentialMap.columns = ["x", "y", "potential"]
print(potentialMap)
potentialMap = potentialMap.pivot(index='x', columns='y', values='potential')

fig = pl.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("Length", size = 18)
# ax.set_zlim(-100.0, 100.0)
ax.set_ylabel("Width", size = 18)
ax.set_zlabel("Potential", size = 18)
xs, ys = np.meshgrid(potentialMap.index, potentialMap.columns, indexing='ij')
ax.plot_surface(xs, ys, potentialMap.values, cmap = "viridis")
pl.show()
