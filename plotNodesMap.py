import pandas as pd
import numpy as np
from matplotlib import pyplot as pl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm


nodesMap = pd.read_csv("./nodesMap.csv", index_col=0)
allNodesInMap = pd.read_csv("./allNodesInMap.csv", index_col=0)
print(nodesMap)
# nodesMap = potentialMap.pivot(index='x', columns='y', values='potential')
startMap = pd.read_csv("./startNode.csv", index_col=0)
targetMap = pd.read_csv("./targetNode.csv", index_col=0)
openListMap = pd.read_csv("./openListMap.csv", index_col=0)
currentNode = pd.read_csv("./currentNode.csv", index_col=0)
trajectoryNodes = pd.read_csv("./trajectoryNodes.csv", index_col=0)


pl.scatter(60 - allNodesInMap.iloc[:, 1], allNodesInMap.iloc[:, 0], marker='x', color='gray')
pl.scatter(60 - nodesMap.iloc[:, 1], nodesMap.iloc[:, 0], color='red')
pl.scatter(60 - openListMap.iloc[:, 1], openListMap.iloc[:, 0], marker='>', alpha=0.7, color='orange')
pl.scatter(60 - currentNode.iloc[:, 1], currentNode.iloc[:, 0], marker='+', s=80, color='green')
colorValues = trajectoryNodes.index.values / trajectoryNodes.index.shape[0]
pl.scatter(60 - trajectoryNodes.iloc[:, 1], trajectoryNodes.iloc[:, 0], marker='^', s=75, c=colorValues, cmap=cm.viridis)
# pl.scatter(startMap.iloc[:, 0], startMap.iloc[:, 1], marker='+')
pl.scatter(60 - targetMap.iloc[:, 1], targetMap.iloc[:, 0], marker='v', color='orange')
pl.show()
