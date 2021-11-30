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
searchedMinPath = pd.read_csv("./searchedMinPath.csv", index_col=0)


pl.scatter(60 - allNodesInMap.iloc[:, 1], allNodesInMap.iloc[:, 0], marker='x', color='gray')
pl.scatter(60 - nodesMap.iloc[:, 1], nodesMap.iloc[:, 0], color='red')
# pl.scatter(60 - openListMap.iloc[:, 1], openListMap.iloc[:, 0], marker='>', alpha=0.7, color='orange')
# pl.scatter(60 - currentNode.iloc[:, 1], currentNode.iloc[:, 0], marker='+', s=80, color='green')
# https://qiita.com/sz_dr/items/d70a6cfd659be6174f4b
colorValues = searchedMinPath.index.values / searchedMinPath.index.shape[0]
pl.scatter(60 - searchedMinPath.iloc[:, 1], searchedMinPath.iloc[:, 0], marker='^', s=75, c=colorValues, cmap=cm.viridis)
pl.scatter(60 - startMap.iloc[:, 1], startMap.iloc[:, 0], marker='+', color='orange')
pl.scatter(60 - targetMap.iloc[:, 1], targetMap.iloc[:, 0], marker='v', color='orange')
pl.show()
