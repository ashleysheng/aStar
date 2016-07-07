from numpy import genfromtxt
import numpy as np
import math
import operator
import time
import heapq as heapq
import _thread
import threading
import tkinter as tk
from tkinter import *
import datetime
import responses
from googlemaps import client as _client
import googlemaps
import requests

#AIzaSyCCA_5XGYNHD3JJLKOK8TEd8IYGjYZD-6Y

#gmaps = googlemaps.Client(key='AIzaSyCCA_5XGYNHD3JJLKOK8TEd8IYGjYZD-6Y')

#responses.add(responses.GET,
#                      'https://maps.googleapis.com/maps/api/elevation/json',
#                      body='{"status":"OK","results":[]}',
#                      status=200,
#                      content_type='application/json')

#results = gmaps.elevation((40.714728, -73.998672))
#responses.add(responses.GET,
#                      'https://maps.googleapis.com/maps/api/elevation/json',
#                      body='{"status":"OK","results":[]}',
#                      status=200,
#                      content_type='application/json')

#path = [(40, -73), (40, -73.01)]

#results2 = gmaps.elevation_along_path(path, 3)

#for each in results2:
#    print(each)






root = tk.Tk()
canvas_1 = Canvas(root,width=1200,height = 1000,background = "white")

route = []      

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
multithread = True
found = False
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

class Example(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.canvas = tk.Canvas(self, width=400,  height=400, 
                                background="bisque")
        self.canvas.pack(fill="both", expand=True)

        graphic1 = GraphicObject(10,10,100,100, name="graphic1")
        graphic2 = GraphicObject(110,110,200,200, name="graphic2")

        graphic1.draw(self.canvas)
        graphic2.draw(self.canvas)

class GraphicObject(object):
    def __init__(self, x0,y0,x1,y1, name=None):
        self.coords = (x0,y0,x1,y1)
        self.name = name

    def draw(self, canvas, outline="black", fill="white"):
        item = canvas.create_oval(self.coords, outline=outline, fill=fill)
        canvas.tag_bind(item, "<1>", self.mouse_click)

    def mouse_click(self, event):
        print ("I got a mouse click (%s)" % self.name)


def multi_thread_monitor():
    global closedSetS
    global closedSetE
    global cameFromE
    global cameFromS
    global notReachable
    global stop
    global meetingPointID
    global multithread
    global found
    if multithread:
        while not found and not notReachable:
            for elem in closedSetE:
                if notReachable:
                    break
                if elem in closedSetS:
                    meetingPointID = elem
                    found = True
                    stop = True
                    continue
    else:
        while not found and not notReachable:
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

def xToLon(xValue):
    return xValue * k1 + b1
def yToLat(yValue):
    return yValue * k2 + b2

def callback(event):

    print ("clicked at", event.x, event.y)


def motion(event):
    global canvas_1
    x, y = event.x, event.y
    #canvas_1.create_oval([x-1,y-1,x+1,y+1],fill="black")
    #print('{}, {}'.format(xToLon(x/10), yToLat(y/10)))

def reconstruct_path(endPointID,needReverse,canvas):
    global canvas_1
    # xf = k1 * xi + b1
    # yf = k2 * yi + b2
    total_path = []
    global multithread, destinationPoint
    if multithread:
        currentID = meetingPointID
    else:
        currentID = coordPointToPointID(lonLatToCoordPoint(destinationPoint))
    if needReverse:
        global cameFromS
        cameFrom = cameFromS
        total_path = [currentID]
        while currentID in cameFrom.keys():
            currentID = cameFrom[currentID]
            total_path.append(currentID)
        total_path.reverse()
        total_path.remove(endPointID)
        #file.write(str(startingPoint.lon)+","+str(startingPoint.lat)+"\n")
        temp = 0
        for each in total_path:
            temp += 1
            coords = pointIDToCoordPoint(each)
            canvas.create_oval([coords.x*10-3,coords.y*10-3,coords.x*10+3,coords.y*10+3],fill="green",outline = "green",activefill="green",activeoutline = "green", activewidth = 10)
            
            lonlat = coordPointToLonlat(coords)
            if temp % 5 == 0:
                xyT = [coords.x*10+20,coords.y*10-1] 
                canvas_1.create_text(xyT, text=str(lonlat.lon)+","+str(lonlat.lat))
            #file.write(str(lonlat.lon)+","+str(lonlat.lat)+"\n")
        total_path.insert(0,endPointID)
    else:
        temp = 0
        global cameFromE
        cameFrom = cameFromE
        while currentID in cameFrom.keys():
            temp += 1
            currentID = cameFrom[currentID]
            coords = pointIDToCoordPoint(currentID)
            canvas.create_oval([coords.x*10-3,coords.y*10-3,coords.x*10+3,coords.y*10+3],fill="green",outline = "green",activefill="green",activeoutline = "green", activewidth = 10 )
            lonlat = coordPointToLonlat(coords)
            if temp % 5 == 0:
                xyT = [coords.x*10+20,coords.y*10-1]
                canvas_1.create_text(xyT, text=str(lonlat.lon)+","+str(lonlat.lat))
            #if currentID != endPointID:
            #    file.write(str(lonlat.lon)+","+str(lonlat.lat)+"\n")
            #else:
            #    file.write(str(destinationPoint.lon)+","+str(destinationPoint.lat)+"\n")
            total_path.append(currentID)

    return total_path



def aStar(startID,goalID,map_data,isStart):
    
    global closedSetE, closedSetS, cameFromE,cameFromS,multithread,found
    global notReachable, errorMessage
    if isStart:
        closedSet = closedSetS
        cameFrom = cameFromS
    else:
        closedSet = closedSetE
        cameFrom = cameFromE

    openSet = []  # IDs
    dicOpenSet = {} 
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
   
        if multithread == False and currentID == goalID:
            found = True
            return 0

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
class rectBoundariesInXY:
    def __init__(self, leftX, rightX, topY,bottomY):
        self.leftX = leftX
        self.rightX= rightX
        self.topY = topY
        self.bottomY = bottomY

def get_boundaries(startingLon,startingLat,destLon,destLat):
    lonDiff = math.fabs(startingLon-destLon)
    latDiff = math.fabs(startingLat-destLat)
    boundaries = rectBoundaries(0,0,0,0)
    if(lonDiff<latDiff):   # lat rules
        boundaries.topLat = max(startingLat,destLat)+1
        boundaries.bottomLat = min(startingLat,destLat)-1
        lonAverage = (startingLon+destLon)/2
        boundaries.leftLon = lonAverage-(latDiff+1)/2
        boundaries.rightLon = lonAverage+(latDiff+1)/2
    else:                   # lon rules
        boundaries.leftLon = min(startingLon,destLon)-1
        boundaries.rightLon = max(startingLon,destLon)+1          # ONLY works in Northern Hemisphere
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



    updated_map_data = map_data

    for y in range (topY, bottomY+1):
        for x in range (leftX, rightX+1):
            checkPoint = coordPoint(x,y)
            checkLonlat = coordPointToLonlat(checkPoint)
            if x>=0 and x< xDimension and y>=0 and x<yDimension and haversine(checkLonlat,center) <= 9:
                updated_map_data[y][x] = 1

    np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 
    
    return rectBoundariesInXY(leftX,rightX,topY,bottomY)



def writeToFile(file,route):
    gmapfile = open("gmapfile.txt", "w")
    index = 0
    file.write(str(index)+"   "+str(startingPoint.lon)+"   "+str(startingPoint.lat)+"   "+"0"+"\n")
    gmapfile.write(str(startingPoint.lon)+","+str(startingPoint.lat)+"\n")
    previousLonLat = startingPoint
    for each in route:
        
        coords = pointIDToCoordPoint(each)
        currlonlat = coordPointToLonlat(coords)
        numOfSubPoints = haversine(previousLonLat,currlonlat) / 0.1
        for i in range (1,int(numOfSubPoints)+1):
            #newLonLat = ((numOfSubPoints-i)*previousLonLat + i*currlonlat)/numOfSubPoints 
            newLonLat = lonLat((numOfSubPoints-i)/numOfSubPoints*previousLonLat.lon+i/numOfSubPoints*currlonlat.lon,(numOfSubPoints-i)/numOfSubPoints*previousLonLat.lat+i/numOfSubPoints*currlonlat.lat)
            index += 1
            file.write(str(index)+"   "+str(newLonLat.lon)+"   "+str(newLonLat.lat)+"   "+"300"+"\n")
            gmapfile.write(str(newLonLat.lon)+","+str(newLonLat.lat)+"\n")

        previousLonLat = currlonlat
    index += 1
    file.write(str(index)+"   "+str(destinationPoint.lon)+"   "+str(destinationPoint.lat)+"   "+"0"+"\n")
    gmapfile.write(str(destinationPoint.lon)+","+str(destinationPoint.lat)+"\n")
    return 0

def find_path(_xDimension, _yDimension, startingLon,startingLat,destLon,destLat):
    loadStart = time.time()
    global root
    #Example(root).pack(fill="both",expand = True)
    #root.mainloop()
    global canvas_1
   # canvas_1 = Canvas(root,width=1200,height = 1000)
    canvas_1.grid(row = 0,column = 1)
    # xf = k1 * xi + b1
    # yf = k2 * yi + b2
    global notReachable,route
    file = open("newfile.txt", "w")
    map_data = genfromtxt('map_data.csv', delimiter=',')

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
    nameTextObjects = []
    for eachline in rows:
        geoData.append([])
        geoData[rowNum].append(eachline[1])
        geoData[rowNum][0] = float(geoData[rowNum][0].replace("RADIUS=",""))
        geoData[rowNum].append(eachline[2])
        geoData[rowNum][1] = float(geoData[rowNum][1].replace("CENTRE=N","")) #lat
        geoData[rowNum].append(eachline[3])
        geoData[rowNum][2] = float(geoData[rowNum][2].replace("W","-")) # lon
        name = ""
        for counter in range (4,len(eachline)):
            name += eachline[counter]
            name += " "
        xyText = [10*lonToX(geoData[rowNum][2]),10*latToY(geoData[rowNum][1])+20] 
        nameObject = canvas_1.create_text(xyText, text=name)
        nameTextObjects.append(nameObject)
        rowNum = rowNum + 1

    img = PhotoImage(file="d.png")

    for eachLine in geoData:
        if eachLine != []:
            rectXY = setUpMap(eachLine,map_data,boundaries)
            if rectXY != 0:
                xy = [10*rectXY.leftX+4, 10*rectXY.topY+4, 10*rectXY.rightX-4, 10*rectXY.bottomY-4] 
                img = PhotoImage(file="z.png")
                #canvas_1.create_arc(xy, start=0, extent=359.999999999,fill = "red",outline = "yellow",activefill = "orange",activeoutline = "orange", activewidth = 15)
                canvas_1.create_arc(xy, start=0, extent=359.999999999,outline = "red",fill = "grey90")
                #canvas_1.create_image(10*rectXY.leftX,10*rectXY.topY, anchor=NW, image=img)
    for each in nameTextObjects:
        canvas_1.tag_raise(each)

                    
    startingID = coordPointToPointID(lonLatToCoordPoint(lonLat(startingLon,startingLat)))
    destID = coordPointToPointID(lonLatToCoordPoint(lonLat(destLon,destLat)))
    startX = pointIDToCoordPoint(startingID).x
    startY = pointIDToCoordPoint(startingID).y
    destX = pointIDToCoordPoint(destID).x
    destY = pointIDToCoordPoint(destID).y
    loadEnd = time.time()
    print("Total Load Time = " + str(loadEnd - loadStart))

    computeStart = time.time()
    if float(map_data[startY][startX]) != 0.0:
        notReachable = True
        errorMessage = "starting point not reachable"
    elif float(map_data[destY][destX]) != 0.0:
        notReachable = True
        errorMessage = "destination point not reachable"
    else:
        distance = haversine(startingPoint,destinationPoint)
        if distance > 15:
            thread1 = threading.Thread(target = aStar, args = (startingID,destID,map_data,True))
            thread2 = threading.Thread(target = aStar, args = (destID,startingID,map_data,False))
            thread3 = threading.Thread(target = multi_thread_monitor,args = ())
            thread1.start()
            thread2.start()
            thread3.start()
            thread1.join()
            thread2.join()
            thread3.join()
        else:
            global multithread
            multithread = False
            thread1 = threading.Thread(target = aStar, args = (startingID,destID,map_data,True))
            thread3 = threading.Thread(target = multi_thread_monitor,args = ())
            thread1.start()
            thread3.start()
            thread1.join()
            thread3.join()

    computeEnd = time.time()

    print("Total Computation Time = " + str(computeEnd - computeStart))
   
    print("Total Time = " + str(loadEnd-loadStart+computeEnd-computeStart))
    if not notReachable:
        route = reconstruct_path(startingID,True,canvas_1) + reconstruct_path(destID,False,canvas_1)
        #print(route)
        writeToFile(file,route)
        totalDis = 0
        for iter in range (0, len(route)-1):
            currCoordPoint = pointIDToCoordPoint(route[iter])
            nextCoordPoint = pointIDToCoordPoint(route[iter+1])
            canvas_1.create_line(10*currCoordPoint.x,10*currCoordPoint.y,10*nextCoordPoint.x,10*nextCoordPoint.y)
            totalDis = totalDis + haversine(coordPointToLonlat(currCoordPoint),coordPointToLonlat(nextCoordPoint))
        file.write("Total Distance  = "+str(totalDis)+" KM")   
        print("Total Distance  = "+str(totalDis)+" KM")                    
        updated_map_data = map_data

        for i in route:
            updated_map_data[pointIDToCoordPoint(i).y][pointIDToCoordPoint(i).x] = 2
        np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 
    else:
        print(errorMessage)

    updated_map_data = map_data
    updated_map_data[lonLatToCoordPoint(startingPoint).y][lonLatToCoordPoint(startingPoint).x] = 3
    updated_map_data[lonLatToCoordPoint(destinationPoint).y][lonLatToCoordPoint(destinationPoint).x] = 3
    np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 

    x_anchor_start = 10*lonLatToCoordPoint(startingPoint).x
    y_anchor_start = 10*lonLatToCoordPoint(startingPoint).y
    x_anchor_end = 10*lonLatToCoordPoint(destinationPoint).x
    y_anchor_end = 10*lonLatToCoordPoint(destinationPoint).y

    xyS = [x_anchor_start, y_anchor_start] 
    startOb = canvas_1.create_text(xyS, text="Starting Point") 
    xyE = [x_anchor_end, y_anchor_end] 
    destOb = canvas_1.create_text(xyE, text="Destination Point")
    root.bind('<Motion>', motion)
    canvas_1.tag_raise(startOb)
    canvas_1.tag_raise(destOb)

   # root.bind("<Button-1>", callback)
    root.mainloop()

    file.close()
    pass





#if __name__ == "__main__":
#root = tk.Tk()
#Example(root).pack(fill="both", expand=True)
#root.mainloop()


#app = Application()                       
#app.master.title('Sample application')    
#app.mainloop()       


startingLon = input("starting point longitude? ")
startingLat = input("starting point latitude? ")
destLon = input("destination point longitude? ")
destLat = input("destination point latitude? ")

find_path(120,120,float(startingLon),float(startingLat),float(destLon),float(destLat))
#find_path(100,100,-80.132752,43.154955 , -79.009258,43.8572)
