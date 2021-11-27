import pandas as pd
import numpy as np
from matplotlib import pyplot as pl
from mpl_toolkits.mplot3d import Axes3D


nodesMap = pd.read_csv("./nodesMap.csv")
print(nodesMap)
# nodesMap = potentialMap.pivot(index='x', columns='y', values='potential')
startMap = pd.read_csv("./startMap.csv")
targetMap = pd.read_csv("./targetMap.csv")
openListMap = pd.read_csv("./openListMap.csv")


pl.scatter(nodesMap['x'], nodesMap['y'])
pl.scatter(openListMap['x'], openListMap['y'], 'X')
pl.scatter(startMap['x'], startMap['y'])
pl.scatter(targetMap['x'], targetMap['y'])
pl.show()
