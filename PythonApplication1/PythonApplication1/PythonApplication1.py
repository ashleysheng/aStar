from numpy import genfromtxt
import numpy as np
import math
import operator
import time
import heapq as heapq
import _thread


k1 = 0
k2 = 0
b1 = 0
b2 = 0
xDimension = 0
yDimension = 0

closedSetS = []
closedSetE = []
cameFromS = []
cameFromE = []
stop = False
confirmStopS = False
confirmStopE = False

class lonLat:
    def __init__(self, lon, lat):
        self.lon = lon
        self.lat = lat

startingPoint = lonLat(0,0)
destinationPoint = lonLat(0,0)

class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def multi_thread_monitor(file,startID,endID):
    global closedSetS
    global closedSetE
    global cameFromE
    global cameFromS
    global stop
    global confirmStopS
    global confirmStopE

    meetingPointID

    found = False
    while not found:
        for elem in closedSetE:
            if elem in closedSetS:
                meetingPointID = elem
                found = True
                stop = True
                continue
    
    done = False

    while not done:
        if confirmStopS and confirmStopE :
            done = True
            return reconstruct_path(cameFrom, meetingPointID,file,startID).apppend(reverse(reconstruct_path(cameFrom, meetingPointID,file,endID)))  
    
    return 0


def coordPointToPointID(coordPoint):
    return coordPoint.x * yDimension + coordPoint.y

def pointIDToCoordPoint(pointID):
    resultPoint = coordPoint(int(pointID/yDimension), pointID % yDimension)
    return resultPoint

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


def coordPointToLonlat(_coords):
    return lonLat(_coords.x * k1 + b1, _coords.y * k2 + b2)

def lonLatToCoordPoint(_lonLat):
    if(k1 != 0 and k2 != 0):
        return coordPoint(int((_lonLat.lon - b1)/k1),int((_lonLat.lat - b2)/k2))


def reconstruct_path(cameFrom, currentID,file,startID):
    # xf = k1 * xi + b1
    # yf = k2 * yi + b2

    total_path = [currentID]
   # coords = pointIDToCoordPoint(currentID)
   # lonlat = coordPointToLonlat(coords)
    file.write(str(destinationPoint.lon)+","+str(destinationPoint.lat)+"\n")
    while currentID in cameFrom.keys():
        currentID = cameFrom[currentID]
        coords = pointIDToCoordPoint(currentID)
        lonlat = coordPointToLonlat(coords)
        if currentID != startID:
            file.write(str(lonlat.lon)+","+str(lonlat.lat)+"\n")
        total_path.append(currentID)
    file.write(str(startingPoint.lon)+","+str(startingPoint.lat)+"\n")
    total_path.reverse()
    return total_path



def aStar(startID,goalID,map_data,file):
    start = time.time()
    closedSet = []
    openSet = []  # IDs
    dicOpenSet = {} 
    openSet.append(startID)
    cameFrom = {}
    gScore = {}   # ID --> score
    fScore = {}
    mapsize = xDimension * yDimension
    for i in range (0, mapsize):
        gScore[i] = math.inf
        fScore[i] = math.inf
    
    gScore[startID] = 0
    fScore[startID] = 0.7 * heuristic_cost(startID,goalID)

    dicOpenSet[startID] = fScore[startID]


    while openSet:
        
        currentID = min(dicOpenSet, key = dicOpenSet.get)  
   
        if currentID == goalID:
            end = time.time()
            print("Total Time", end= " ")
            print( end - start)
            return reconstruct_path(cameFrom, currentID,file,startID)


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
                fScore[eachAdjacentNeighbour] = gScore[eachAdjacentNeighbour] + 0.7 * heuristic_cost(eachAdjacentNeighbour,goalID)
                dicOpenSet[eachAdjacentNeighbour] = fScore[eachAdjacentNeighbour]
     
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
                fScore[eachCornerNeighbour] = gScore[eachCornerNeighbour] + 0.7 * heuristic_cost(eachCornerNeighbour,goalID)
                dicOpenSet[eachCornerNeighbour] = fScore[eachCornerNeighbour]
        
    return False

class rectBoundaries:
    def __init__(self, leftLon, rightLon, topLat,bottomLat):
        self.leftLon = leftLon
        self.rightLon = rightLon
        self.topLat = topLat
        self.bottomLat = bottomLat

def get_boundaries(startingLon,startingLat,destLon,destLat):
    lonDiff = math.fabs(startingLon-destLon)
    latDiff = math.fabs(startingLat-destLat)
    boundaries = rectBoundaries(0,0,0,0)
    if(lonDiff<latDiff):   # lat rules
        boundaries.topLat = max(startingLat,destLat)+0.5
        boundaries.bottomLat = min(startingLat,destLat)-0.5
        lonAverage = (startingLon+destLon)/2
        boundaries.leftLon = lonAverage-(latDiff+1)/2
        boundaries.rightLon = lonAverage+(latDiff+1)/2
    else:                   # lon rules
        boundaries.leftLon = min(startingLon,destLon)-0.5
        boundaries.rightLon = max(startingLon,destLon)+0.5          # ONLY works in Northern Hemisphere
        latAverage = (startingLat+destLat)/2
        boundaries.topLat = latAverage + (lonDiff+1)/2
        boundaries.bottomLat = latAverage - (lonDiff+1)/2
    return boundaries

def find_path(_xDimension, _yDimension, startingLon,startingLat,destLon,destLat):
    # xf = k1 * xi + b1
    # yf = k2 * yi + b2

    file = open("newfile.txt", "w")
    map_data = genfromtxt('map_data.csv', delimiter=',')

    print("Clean the map or draw a route")
    response = input("Clean the map or draw a route (1 or 2)")

    if(response == "1"):
        np.savetxt("updated_data.csv", map_data ,fmt='%d', delimiter=',') 

    else:
        boundaries = get_boundaries(startingLon,startingLat,destLon,destLat)

        global k1, k2, b1, b2, xDimension, yDimension, startingPoint, destinationPoint
        startingPoint.lon = startingLon
        startingPoint.lat = startingLat
        destinationPoint.lat = destLat
        destinationPoint.lon = destLon
        xDimension = _xDimension
        yDimension = _yDimension
        k1 = (boundaries.rightLon - boundaries.leftLon)/(xDimension-1)
        k2 = (boundaries.bottomLat - boundaries.topLat)/(yDimension-1)
        b1 = boundaries.leftLon
        b2 = boundaries.topLat
        
        startingID = coordPointToPointID(lonLatToCoordPoint(lonLat(startingLon,startingLat)))
        destID = coordPointToPointID(lonLatToCoordPoint(lonLat(destLon,destLat)))
        route = aStar(startingID,destID,map_data,file)
        print(route)
        updated_map_data = map_data

        for i in route:
            updated_map_data[pointIDToCoordPoint(i).y][pointIDToCoordPoint(i).x] = 2
        np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 

    file.close()
    pass

#def find_path(_x,_y,startingLon,startingLat,destLon,destLat)





find_path(100,100,-82.243164,47.749019,-88.275146,52.032225)