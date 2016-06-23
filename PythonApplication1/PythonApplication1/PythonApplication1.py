from numpy import genfromtxt
import numpy as np
import math
import operator
import time
import heapq as heapq

xDimension = 100
yDimension = 100

class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y


'''***********************************************************************'''
def coordPointToPointID(coordPoint):
    return coordPoint.x * yDimension + coordPoint.y

def pointIDToCoordPoint(pointID):
    resultPoint = coordPoint(int(pointID/yDimension), pointID % yDimension)
    return resultPoint
'''***********************************************************************'''



def heuristic_cost(start,goal):
    startPoint = pointIDToCoordPoint(start)
    goalPoint = pointIDToCoordPoint(goal)
    xDiff = math.fabs(startPoint.x-goalPoint.x)
    yDiff = math.fabs(startPoint.y-goalPoint.y)
    result = math.fabs(xDiff - yDiff) + min(xDiff,yDiff) * 1.414
    return result 

def getAdjacentNeighbours(currentID,map_data):

    mapsize = xDimension * yDimension
    left = currentID - yDimension # IDs
    right = currentID + yDimension
    top = currentID - 1
    bottom = currentID + 1
    currentX = pointIDToCoordPoint(currentID).x
    currentY = pointIDToCoordPoint(currentID).y
    
    if currentX == 0 and currentY == 0:                             # top left corner
        potentialNeightbours = [right, bottom]
    elif currentX == 0 and currentY == yDimension - 1:              # bottom left corner
        potentialNeightbours = [right, top]
    elif currentX == xDimension - 1 and currentY == 0:              # top right corner
        potentialNeightbours = [left, bottom]
    elif currentX == xDimension - 1 and currentY == yDimension - 1: # bottom right corner
        potentialNeightbours = [left, top]
    elif currentX == 0:                                             # left edge
        potentialNeightbours = [right, top, bottom]
    elif currentX == xDimension - 1:                                # right edge
        potentialNeightbours = [left, top, bottom]
    elif currentY == 0:                                             # top edge
        potentialNeightbours = [left, right, bottom]
    elif currentY == yDimension - 1:                                # bottom edge
        potentialNeightbours = [left, right, top]
    else:
        potentialNeightbours = [left, right, top, bottom]
    
    adjacentNeighbours = []
    for each in potentialNeightbours:
     #   if each >= 0 and each < mapsize:
            coords = pointIDToCoordPoint(each)
        #    print(map_data[coords.x][coords.y])
            if float(map_data[coords.y][coords.x]) == 0.0:
                adjacentNeighbours.append(each)
            else:
                continue

    return adjacentNeighbours

def getCornerNeighbours(currentID,map_data):
    mapsize = xDimension * yDimension
    topLeft = currentID - yDimension - 1
    bottomLeft = currentID - yDimension + 1
    topRight = currentID + yDimension - 1
    bottomRight = currentID + yDimension + 1
    currentX = pointIDToCoordPoint(currentID).x
    currentY = pointIDToCoordPoint(currentID).y

    if currentX == 0 and currentY == 0:                             # top left corner
        potentialNeightbours = [bottomRight]
    elif currentX == 0 and currentY == yDimension - 1:              # bottom left corner
        potentialNeightbours = [topRight]
    elif currentX == xDimension - 1 and currentY == 0:              # top right corner
        potentialNeightbours = [bottomLeft]
    elif currentX == xDimension - 1 and currentY == yDimension - 1: # bottom right corner
        potentialNeightbours = [topLeft]
    elif currentX == 0:                                             # left edge
        potentialNeightbours = [topRight, bottomRight]
    elif currentX == xDimension - 1:                                # right edge
        potentialNeightbours = [topLeft, bottomLeft]
    elif currentY == 0:                                             # top edge
        potentialNeightbours = [bottomLeft, bottomRight]
    elif currentY == yDimension - 1:                                # bottom edge
        potentialNeightbours = [topLeft, topRight]
    else:
        potentialNeightbours = [topLeft, bottomLeft, topRight, bottomRight]
    
    cornerNeighbours = []
    for each in potentialNeightbours:
        if each >= 0 and each < mapsize:
            coords = pointIDToCoordPoint(each)
            if float(map_data[coords.y][coords.x]) == 0.0:
                cornerNeighbours.append(each)
            else:
                continue
    return cornerNeighbours

def reconstruct_path(cameFrom, currentID):
    total_path = [currentID]
    while currentID in cameFrom.keys():
        currentID = cameFrom[currentID]
        total_path.append(currentID)
    total_path.reverse()
    return total_path



def aStar(startID,goalID,map_data):
    start = time.time()
    print("start Time", end= " ")
    print(start)
    closedSet = []
    openSet = []  # IDs
    dicOpenSet = {} 
    openSet.append(startID)
    cameFrom = {}
    gScore = {}   # ID --> score
    fScore = {}
    mapsize = xDimension * yDimension
    for i in range (0, mapsize):
        gScore[i] = 99999
        fScore[i] = 99999
    
    gScore[startID] = 0
    fScore[startID] = heuristic_cost(startID,goalID)

    dicOpenSet[startID] = fScore[startID]


    while openSet:
        
        currentID = min(dicOpenSet, key = dicOpenSet.get)  
   
        #tupleFScore = sorted(fScore.items(), key=operator.itemgetter(1)) # sort by fScore low to high
      #  count = 0
       # currentID = tupleFScore[count][0] # ID with lowest fScore
  
       # while currentID not in openSet:
        #    count = count + 1
        #    currentID = tupleFScore[count][0] # ID with lowest fScore

        if currentID == goalID:
            end = time.time()
            print("Total Time", end= " ")
            print( end - start)
            return reconstruct_path(cameFrom, currentID)


        openSet.remove(currentID)
        del dicOpenSet[currentID]
        

        if currentID not in closedSet:
            closedSet.append(currentID)
        adjacentNeighbours = getAdjacentNeighbours(currentID,map_data)
        for eachAdjacentNeighbour in adjacentNeighbours:
            if eachAdjacentNeighbour in closedSet:
                continue

            tentative_gScore = gScore[currentID] + 1

         
            if eachAdjacentNeighbour not in openSet:
                openSet.append(eachAdjacentNeighbour)

            if eachAdjacentNeighbour not in openSet:
                dicOpenSet[eachAdjacentNeighbour] = fScore[eachAdjacentNeighbour]
            
            if tentative_gScore >= gScore[eachAdjacentNeighbour]:
                continue
            else:
                cameFrom[eachAdjacentNeighbour] = currentID
                gScore[eachAdjacentNeighbour] = tentative_gScore
                fScore[eachAdjacentNeighbour] = gScore[eachAdjacentNeighbour] + heuristic_cost(eachAdjacentNeighbour,goalID) * 0.5
                dicOpenSet[eachAdjacentNeighbour] = fScore[eachAdjacentNeighbour]
        #    fScore[eachAdjacentNeighbour] = gScore[eachAdjacentNeighbour]
     
        cornerNeighbours = getCornerNeighbours(currentID,map_data)
        for eachCornerNeighbour in cornerNeighbours:
            if eachCornerNeighbour in closedSet:
                continue
            tentative_gScore = gScore[currentID] + 1.414
            if eachCornerNeighbour not in openSet:
                openSet.append(eachCornerNeighbour)
    
            if eachCornerNeighbour not in openSet:
                 dicOpenSet[eachCornerNeighbour] = fScore[eachCornerNeighbour]
                 
            if tentative_gScore >= gScore[eachCornerNeighbour]:
                continue
            else:
                cameFrom[eachCornerNeighbour] = currentID
                gScore[eachCornerNeighbour] = tentative_gScore
                fScore[eachCornerNeighbour] = gScore[eachCornerNeighbour] + heuristic_cost(eachCornerNeighbour,goalID)  * 0.5
                dicOpenSet[eachCornerNeighbour] = fScore[eachCornerNeighbour]
           # fScore[eachCornerNeighbour] = gScore[eachCornerNeighbour]
        
    return False




print("This line will be printed.")
map_data = genfromtxt('map_data.csv', delimiter=',')
startPoint = coordPoint(0,0)
goalPoint = coordPoint(99,99)

#print(map_data[0][0])
#print(pointIDToCoordPoint(100))
#print(pointIDToCoordPoint(99))



print(getAdjacentNeighbours(9898,map_data))
print(getCornerNeighbours(9898,map_data))


print("Clean the map or draw a route")
response = input("Clean the map or draw a route (1 or 2)")

if(response == "1"):
    np.savetxt("updated_data.csv", map_data ,fmt='%d', delimiter=',') 

else:
    print(aStar(0,305,map_data))
    route = aStar(0,9999,map_data)
    updated_map_data = map_data

    for i in route:
        updated_map_data[pointIDToCoordPoint(i).y][pointIDToCoordPoint(i).x] = 2
        #print (pointIDToCoordPoint(i).x, end = '')
        #print (pointIDToCoordPoint(i).y, end = " " )
        #print (updated_map_data[pointIDToCoordPoint(i).y][pointIDToCoordPoint(i).x])

    np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 