from numpy import genfromtxt
import numpy as np
import math
np.set_printoptions(threshold=np.nan)

class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def heuristic_cost(start,goal):
    return 0 


def aStar(start,goal,mapsize):
    
    closedSet = []
    openSet = []
    heappush(openSet,start)
    cameFrom = []
    dist= []
    dist[start] = 0
    fscore = []
    
    for i in mapsize:
        dist.append[math.inf]
    for i in mapsize:
        fscore.append[math.inf]
    
    dist[start] = 0


    
    

    
    
        
    




    pass

'''***********************************************************************'''
def coordPointToPointID(coordPoint):
    return coordPoint.x * 100 + coordPoint.y

def pointIDToCoordPoint(pointID):
    resultPoint = coordPoint(int(pointID/100), pointid % 100)
    return 
'''***********************************************************************'''



print("This line will be printed.")
map_data = genfromtxt('map_data.csv', delimiter=',')
startPoint = coordPoint(0,0)
goalPoint = coordPoint(99,99)

print(map_data[startPoint.x][startPoint.y])

# print(map_data[startPoint.x][startPoint.y])



