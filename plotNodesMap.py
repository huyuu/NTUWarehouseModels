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
currentNode = pd.read_csv("./currentNode.csv", index_col=0)
trajectoryNodes = pd.read_csv("./trajectoryNodes.csv", index_col=0)


pl.scatter(allNodesInMap.iloc[:, 1], 60 - allNodesInMap.iloc[:, 0], marker='x', color='gray')
pl.scatter(nodesMap.iloc[:, 1], 60 - nodesMap.iloc[:, 0], color='red')
pl.scatter(openListMap.iloc[:, 1], 60 - openListMap.iloc[:, 0], marker='>', alpha=0.7, color='orange')
pl.scatter(currentNode.iloc[:, 1], 60 - currentNode.iloc[:, 0], marker='+', s=80, color='green')
pl.scatter(trajectoryNodes.iloc[:, 1], 60 - trajectoryNodes.iloc[:, 0], marker='^', s=75, color='black')
# pl.scatter(startMap.iloc[:, 0], startMap.iloc[:, 1], marker='+')
pl.scatter(targetMap.iloc[:, 1], 60 - targetMap.iloc[:, 0], marker='v', color='orange')
pl.show()
