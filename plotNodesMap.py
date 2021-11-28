import pandas as pd
import numpy as np
from matplotlib import pyplot as pl
from mpl_toolkits.mplot3d import Axes3D


nodesMap = pd.read_csv("./nodesMap.csv", index_col=0)
allNodesInMap = pd.read_csv("./allNodesInMap.csv", index_col=0)
print(nodesMap)
# nodesMap = potentialMap.pivot(index='x', columns='y', values='potential')
startMap = pd.read_csv("./startNode.csv", index_col=0)
targetMap = pd.read_csv("./targetNode.csv", index_col=0)
openListMap = pd.read_csv("./openListMap.csv", index_col=0)


pl.scatter(nodesMap.iloc[:, 0], nodesMap.iloc[:, 1])
pl.scatter(allNodesInMap.iloc[:, 0], allNodesInMap.iloc[:, 1], marker='x')
pl.scatter(openListMap.iloc[:, 0], openListMap.iloc[:, 1], marker='O', alpha=0.7)
pl.scatter(startMap.iloc[:, 0], startMap.iloc[:, 1], marker='+')
pl.scatter(targetMap.iloc[:, 0], targetMap.iloc[:, 1], marker='-')
pl.show()
