from numpy import genfromtxt
import numpy as np
import math
import operator
import time
import heapq as heapq

import  _thread 
import time



class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class pair:
    def __init__(self, id, score):
        self.id = id
        self.score = score

    def __lt__(self, other): 
        return self.score < other.score
       
def Bi_directional_path_combinator 


def coordPointToPointID(coordPoint,yDimension):
    return coordPoint.x * yDimension + coordPoint.y

def pointIDToCoordPoint(pointID,yDimension):
    resultPoint = coordPoint(int(pointID/yDimension), pointID % yDimension)
    return resultPoint

def heuristic_cost(start,goal,yDimension):
    startPoint = pointIDToCoordPoint(start,yDimension)
    goalPoint = pointIDToCoordPoint(goal,yDimension)
    xDiff = math.fabs(startPoint.x-goalPoint.x)
    yDiff = math.fabs(startPoint.y-goalPoint.y)
    result = math.fabs(xDiff - yDiff) + min(xDiff,yDiff) * 1.414
    return result 

def getAdjacentNeighbours(currentID,map_data,xDimension,yDimension):

    mapsize = xDimension * yDimension
    left = currentID - yDimension # IDs
    right = currentID + yDimension
    top = currentID - 1
    bottom = currentID + 1
    currentX = pointIDToCoordPoint(currentID,yDimension).x
    currentY = pointIDToCoordPoint(currentID,yDimension).y
    
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
            coords = pointIDToCoordPoint(each,yDimension)
        #    print(map_data[coords.x][coords.y])
            if float(map_data[coords.y][coords.x]) == 0.0:
                adjacentNeighbours.append(each)
            else:
                continue

    return adjacentNeighbours

def getCornerNeighbours(currentID,map_data,xDimension,yDimension):
    mapsize = xDimension * yDimension
    topLeft = currentID - yDimension - 1
    bottomLeft = currentID - yDimension + 1
    topRight = currentID + yDimension - 1
    bottomRight = currentID + yDimension + 1
    currentX = pointIDToCoordPoint(currentID,yDimension).x
    currentY = pointIDToCoordPoint(currentID,yDimension).y

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
            coords = pointIDToCoordPoint(each,yDimension)
            if float(map_data[coords.y][coords.x]) == 0.0:
                cornerNeighbours.append(each)
            else:
                continue
    return cornerNeighbours

def reconstruct_path(cameFrom, currentID,file,yDimension,leftLon,rightLon,topLat,bottomLat):
    total_path = [currentID]
    while currentID in cameFrom.keys():
        currentID = cameFrom[currentID]
        coords = pointIDToCoordPoint(currentID,yDimension)
        file.write(str(coords.x*0.00096268-89.31105268)+","+str(-coords.y*0.0005498969+48.41687989)+"\n")
        total_path.append(currentID)
    total_path.reverse()
    return total_path



def aStar(startID,goalID,map_data,file,xDimension,yDimension,leftLon,rightLon,topLat,bottomLat):
    start = time.time()
    closedSet = []
    openSet = []  # IDs
    dicOpenSet = {} 
    listOpenSet = []

    openSet.append(startID)
    cameFrom = {}
    gScore = {}   # ID --> score
    fScore = {}
    mapsize = xDimension * yDimension
    for i in range (0, mapsize):
        gScore[i] = math.inf
        fScore[i] = math.inf
    
    gScore[startID] = 0
    fScore[startID] = 0.7 * heuristic_cost(startID,goalID,yDimension)

    dicOpenSet[startID] = fScore[startID]
    
    heapq.heapify(listOpenSet)
    heapq.heappush(listOpenSet,pair(startID,fScore[startID]))

    while openSet:
        
        currentID = min(dicOpenSet, key = dicOpenSet.get)
 
   
        if currentID == goalID:
            end = time.time()
            print("Total Time", end= " ")
            print( end - start)
            return reconstruct_path(cameFrom, currentID,file,yDimension,leftLon,rightLon,topLat,bottomLat)


        openSet.remove(currentID)
        del dicOpenSet[currentID]
        

        if currentID not in closedSet:
            closedSet.append(currentID)
        adjacentNeighbours = getAdjacentNeighbours(currentID,map_data,xDimension,yDimension)
        for eachAdjacentNeighbour in adjacentNeighbours:
            if eachAdjacentNeighbour in closedSet:
                continue

            tentative_gScore = gScore[currentID] + 1

            if eachAdjacentNeighbour not in openSet:
                openSet.append(eachAdjacentNeighbour)

            if tentative_gScore >= gScore[eachAdjacentNeighbour]:
                continue
            else:
                cameFrom[eachAdjacentNeighbour] = currentID
                gScore[eachAdjacentNeighbour] = tentative_gScore
                fScore[eachAdjacentNeighbour] = gScore[eachAdjacentNeighbour] + 0.7 * heuristic_cost(eachAdjacentNeighbour,goalID,yDimension)
                dicOpenSet[eachAdjacentNeighbour] = fScore[eachAdjacentNeighbour]
     
        cornerNeighbours = getCornerNeighbours(currentID,map_data,xDimension,yDimension)
        for eachCornerNeighbour in cornerNeighbours:
            if eachCornerNeighbour in closedSet:
                continue
            tentative_gScore = gScore[currentID] + 1.414

            if eachCornerNeighbour not in openSet:
                openSet.append(eachCornerNeighbour)

                 
            if tentative_gScore >= gScore[eachCornerNeighbour]:
                continue
            else:
                cameFrom[eachCornerNeighbour] = currentID
                gScore[eachCornerNeighbour] = tentative_gScore
                fScore[eachCornerNeighbour] = gScore[eachCornerNeighbour] + 0.7 * heuristic_cost(eachCornerNeighbour,goalID,yDimension)
                dicOpenSet[eachCornerNeighbour] = fScore[eachCornerNeighbour]
        
    return False


def find_path(_xDimension,_yDimension,leftLon,rightLon,topLat,bottomLat):
    xDimension = _xDimension
    yDimension = _yDimension
    file = open("newfile.txt", "w")
    map_data = genfromtxt('map_data.csv', delimiter=',')

    print("Clean the map or draw a route")
    response = input("Clean the map or draw a route (1 or 2)")

    if(response == "1"):
        np.savetxt("updated_data.csv", map_data ,fmt='%d', delimiter=',') 

    else:
        route = aStar(0,9999,map_data,file,xDimension,yDimension,leftLon,rightLon,topLat,bottomLat)
        print(route)
        updated_map_data = map_data

        for i in route:
            updated_map_data[pointIDToCoordPoint(i,yDimension).y][pointIDToCoordPoint(i,yDimension).x] = 2
        np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 

    file.close()
    pass


a = pair(40,60)
b = pair(30,120)
print(a < b)
find_path(100,100,1,2,3,4)


