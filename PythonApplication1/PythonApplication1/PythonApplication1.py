from numpy import genfromtxt
import numpy as np
import math
import operator
import time
import heapq as heapq
import _thread
import threading

EARTH_RADIUS = 6371
k1 = 0
k2 = 0
b1 = 0
b2 = 0
xDimension = 0
yDimension = 0
notReachable = False
errorMessage = ""
closedSetS = []
closedSetE = []
cameFromS = {}
cameFromE = {}
stop = False
meetingPointID = 0
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

def multi_thread_monitor():
    start = time.time()
    global closedSetS
    global closedSetE
    global cameFromE
    global cameFromS
    global notReachable
    global stop
    global meetingPointID
    found = False

    while not found and not notReachable:
        for elem in closedSetE:
            if notReachable:
                break
            if elem in closedSetS:
                meetingPointID = elem
                found = True
                stop = True
                continue
    
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
        coords = pointIDToCoordPoint(each)
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

def lonToX(longitude):
    if (k1 != 0):
        return int((longitude - b1)/k1)
    
def latToY(latitude):
    if(k2 != 0):
        return int((latitude - b2)/k2)

def reconstruct_path(endPointID,file,needReverse):
    # xf = k1 * xi + b1
    # yf = k2 * yi + b2
    total_path = []
    currentID = meetingPointID
    if needReverse:
        global cameFromS
        cameFrom = cameFromS
        total_path = [currentID]
        while currentID in cameFrom.keys():
            currentID = cameFrom[currentID]
            total_path.append(currentID)
        total_path.reverse()
        total_path.remove(endPointID)
        file.write(str(startingPoint.lon)+","+str(startingPoint.lat)+"\n")
        for each in total_path:
            coords = pointIDToCoordPoint(each)
            lonlat = coordPointToLonlat(coords)
            file.write(str(lonlat.lon)+","+str(lonlat.lat)+"\n")
            
    else:
        global cameFromE
        cameFrom = cameFromE
        while currentID in cameFrom.keys():
            currentID = cameFrom[currentID]
            coords = pointIDToCoordPoint(currentID)
            lonlat = coordPointToLonlat(coords)
            if currentID != endPointID:
                file.write(str(lonlat.lon)+","+str(lonlat.lat)+"\n")
            else:
                file.write(str(destinationPoint.lon)+","+str(destinationPoint.lat)+"\n")
            total_path.append(currentID)

    return total_path



def aStar(startID,goalID,map_data,isStart):
    
    start = time.time()
    global closedSetE, closedSetS, cameFromE,cameFromS 
    global notReachable, errorMessage
    if isStart:
        closedSet = closedSetS
        cameFrom = cameFromS
    else:
        closedSet = closedSetE
        cameFrom = cameFromE

    openSet = []  # IDs
    dicOpenSet = {} 
    startX = pointIDToCoordPoint(startID).x
    startY = pointIDToCoordPoint(startID).y
    if float(map_data[startY][startX]) != 0.0:
        notReachable = True
        errorMessage = "starting point/destination point not reachable"
        return 0
    openSet.append(startID)
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
        if stop: 
            return 0
        currentID = min(dicOpenSet, key = dicOpenSet.get)  
   
        if currentID == goalID:
            end = time.time()
            print("Total Time", end= " ")
            print( end - start)

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
      
    notReachable = True
    errorMessage = "path not found"
    return 0

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


def haversine(lonlat1,lonlat2):
    global EARTH_RADIUS
    dlon = math.radians(math.fabs(lonlat1.lon - lonlat2.lon))
    dlat = math.radians(math.fabs(lonlat1.lat - lonlat2.lat))
    a = math.pow(math.sin(dlat/2),2) + math.cos(math.radians(lonlat1.lat)) * math.cos(math.radians(lonlat2.lat)) * math.pow(math.sin(dlon/2),2)
    c = 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))
    distance = EARTH_RADIUS *c
    return distance

def get_bound_point(lonlatcenter,lonlatlower,lonlatupper,keepLon,radius):
    if keepLon:
        lonlatmid = lonLat(lonlatlower.lon, (lonlatlower.lat+lonlatupper.lat)/2)
    else:
        lonlatmid = lonLat((lonlatlower.lon+lonlatupper.lon)/2, lonlatlower.lat)
    distance = haversine(lonlatcenter,lonlatmid)
    if distance - radius < 0.001 and distance - radius >= 0:
        return lonlatmid
    if distance > radius:
        return get_bound_point(lonlatcenter,lonlatlower,lonlatmid,keepLon,radius)
    if distance < radius:
        return get_bound_point(lonlatcenter,lonlatmid,lonlatupper,keepLon,radius)
    

def setUpMap(line,map_data, boundaries):
    global EARTH_RADIUS, xDimension,yDimension

    centerLat = line[1]
    centerLon = line[2]
    radius = line[0]
    center = lonLat(centerLon,centerLat)
    # (x,y) = (lon * cos(lat avg) , lat)
    # d = R * sqrt((y2-y1)^2 + (x2-x1)^2)

    if boundaries.bottomLat > centerLat:
        return 0
    elif boundaries.topLat < centerLat:
        return 0
    elif boundaries.leftLon > centerLon:
        return 0
    elif boundaries.rightLon < centerLon:
        return 0

    k = radius/EARTH_RADIUS
    topLat = centerLat + k 
    bottomLat = centerLat - k
    q =  math.cos(centerLat/180*math.pi)
    leftLon = (centerLon * q-k)/q
    rightLon = (centerLon * q+k)/q

    topZoneLat = get_bound_point(lonLat(centerLon,centerLat),lonLat(centerLon,centerLat),lonLat(centerLon,centerLat+0.3),True,radius).lat
    bottomZoneLat = get_bound_point(lonLat(centerLon,centerLat),lonLat(centerLon,centerLat),lonLat(centerLon,centerLat-0.3),True,radius).lat
    rightZoneLon = get_bound_point(lonLat(centerLon,centerLat),lonLat(centerLon,centerLat),lonLat(centerLon+0.3,centerLat),False,radius).lon
    leftZoneLon = get_bound_point(lonLat(centerLon,centerLat),lonLat(centerLon,centerLat),lonLat(centerLon-0.3,centerLat),False,radius).lon
    
    topY = latToY(topZoneLat)
    bottomY = latToY(bottomZoneLat)
    rightX = lonToX(rightZoneLon)
    leftX = lonToX(leftZoneLon)

    print(topY)
    print(bottomY)
    print(rightX)
    print(leftX)

    updated_map_data = map_data

    for y in range (topY, bottomY+1):
        for x in range (leftX, rightX+1):
            checkPoint = coordPoint(x,y)
            checkLonlat = coordPointToLonlat(checkPoint)
            if x>=0 and x< xDimension and y>=0 and x<yDimension and haversine(checkLonlat,center) <= 9:
                updated_map_data[y][x] = 1

    np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 
    
    return 0

def find_path(_xDimension, _yDimension, startingLon,startingLat,destLon,destLat):

    # xf = k1 * xi + b1
    # yf = k2 * yi + b2
    global notReachable
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
        k1 = (boundaries.rightLon - boundaries.leftLon)/(xDimension)
        k2 = (boundaries.bottomLat - boundaries.topLat)/(yDimension)
        b1 = boundaries.leftLon
        b2 = boundaries.topLat

        geoDataFile = open("NFZ data.txt","r")
        rows = (row.strip().split() for row in geoDataFile)
        geoData = [[]]

        rowNum = 0
        for eachline in rows:
            geoData.append([])
            geoData[rowNum].append(eachline[1])
            geoData[rowNum][0] = float(geoData[rowNum][0].replace("RADIUS=",""))
            geoData[rowNum].append(eachline[2])
            geoData[rowNum][1] = float(geoData[rowNum][1].replace("CENTRE=N",""))/10000
            geoData[rowNum].append(eachline[3])
            geoData[rowNum][2] = float(geoData[rowNum][2].replace("W","-"))/10000
            rowNum = rowNum + 1

        for eachLine in geoData:
            if eachLine != []:
                print(eachLine)
                setUpMap(eachLine,map_data,boundaries)

        
        startingID = coordPointToPointID(lonLatToCoordPoint(lonLat(startingLon,startingLat)))
        destID = coordPointToPointID(lonLatToCoordPoint(lonLat(destLon,destLat)))

        thread1 = threading.Thread(target = aStar, args = (startingID,destID,map_data,True))
        thread2 = threading.Thread(target = aStar, args = (destID,startingID,map_data,False))
        thread3 = threading.Thread(target = multi_thread_monitor,args = ())
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()

        print("finished")
        
        if not notReachable:
            route = reconstruct_path(startingID,file,True) + reconstruct_path(destID,file,False)
            updated_map_data = map_data

            for i in route:
                updated_map_data[pointIDToCoordPoint(i).y][pointIDToCoordPoint(i).x] = 2
            np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 
        else:
            print(errorMessage)
    file.close()
    pass

#def find_path(_x,_y,startingLon,startingLat,destLon,destLat)
find_path(100,100,-77.243164,42.749019,-80.275146,44.032225)