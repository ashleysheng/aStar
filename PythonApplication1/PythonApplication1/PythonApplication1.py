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
from tkinter import messagebox
import datetime
from tkinter import Canvas
import responses
from googlemaps import client as _client
import googlemaps
import requests
import decimal 
from decimal import *
import srtm
from plotting import *


import folium
map_0 = folium.Map()
map_osm = folium.Map()
elevation_data = srtm.get_data()
root = tk.Tk()
import ttk
progressbar = ttk.Progressbar(orient=HORIZONTAL, length=800, mode='determinate')
progressbar.grid(row =5,column = 0,columnspan = 4)
progressbar.start()
root.wm_title("DDC Flight Path Finding")
canvas_1 = Canvas(width=1000,height = 800,background = "white")
photo = PhotoImage()
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
underFifteen = True
found = False
meetingPointID = 0
UIZoomFactor = 6.5
loadStart = 0.0
fileName = "DDCRoute"
geoData = [[]]
flatModeOn = False
standardFinished = False
destinationPointAltitude = 0.0
startingPointAltitude = 0.0
upperAltitude=0.0
lowerAltitude=0.0
altitude = 300.00
droneTakeOffPhoto = PhotoImage()
droneLandingPhoto = PhotoImage()
elevationArray=[]
upperBoundArray=[]
lowerBoundArray=[]
maxStep = 50.0
maxStepSpecified = False
indexArray = []            #exclude starting and ending point since elevation = 0
lonArray = []
latArray = []
class lonLat:
    def __init__(self, lon, lat):
        self.lon = lon
        self.lat = lat

class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
        
startingPoint = lonLat(0,0)
destinationPoint = lonLat(0,0)



def multi_thread_monitor():
    global closedSetS, closedSetE,cameFromE,cameFromS,notReachable
    global stop,meetingPointID,multithread,found
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

def coordPointToLonLat(_coords):
    return lonLat(_coords.x * k1 + b1, _coords.y * k2 + b2)

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
    #global flatModeOn,startingPointAltitude,destinationPointAltitude
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
    
    #updated_map_data = genfromtxt('updated_data.csv', delimiter=',')

    currAltInMeters = elevation_data.get_elevation( coordPointToLonLat(pointIDToCoordPoint(currentID)).lat,coordPointToLonLat(pointIDToCoordPoint(currentID)).lon)
    if currAltInMeters:
        currAlt = currAltInMeters * 3.28084
    else:
        print("no data\n") 
    global upperAltitude,lowerAltitude
    adjacentNeighbours = []
    #for each in potentialNeightbours:
    #    coords = pointIDToCoordPoint(each)
    #    if float(map_data[coords.y][coords.x]) == 0.0:
    #        adjacentNeighbours.append(each)
    #    else:
    #        continue
    for each in potentialNeightbours:
        coords = pointIDToCoordPoint(each)
        
        if float(map_data[coords.y][coords.x]) == 0.0:
            if not flatModeOn:                                               # standard operation
                adjacentNeighbours.append(each)
            else:                                                           # flat
                eachLonLat = coordPointToLonLat(coords)
                neighbourAltInMeters = elevation_data.get_elevation( eachLonLat.lat,eachLonLat.lon)
                if neighbourAltInMeters:
                    neighbourAlt = neighbourAltInMeters*3.28084
                    global maxStep
                    if  lowerAltitude <= neighbourAlt <= upperAltitude and math.fabs(currAlt-neighbourAlt)<maxStep:
                        adjacentNeighbours.append(each)
                else:
                     print("no data\n") 
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
        
    currAltInMeters = elevation_data.get_elevation( coordPointToLonLat(pointIDToCoordPoint(currentID)).lat,coordPointToLonLat(pointIDToCoordPoint(currentID)).lon)
    if currAltInMeters:
        currAlt = currAltInMeters * 3.28084
    else:
        print("no data\n") 

    cornerNeighbours = []
    #for each in potentialNeightbours:
    #    if each >= 0 and each < mapsize:
    #        coords = pointIDToCoordPoint(each)
    #        if float(map_data[coords.y][coords.x]) == 0.0:                          
    #            cornerNeighbours.append(each)
    #        else:
    #            continue 

    #updated_map_data = genfromtxt('updated_data.csv', delimiter=',')

    global flatModeOn,startingPointAltitude,destinationPointAltitude
    global upperAltitude,lowerAltitude
    for each in potentialNeightbours:
        if each >= 0 and each < mapsize:
            coords = pointIDToCoordPoint(each)
            if float(map_data[coords.y][coords.x]) == 0.0:
                if not flatModeOn:                                               # standard operation
                    cornerNeighbours.append(each)
                else:                                                           # flat
                    eachLonLat = coordPointToLonLat(coords)
                    neighbourAltInMeters = elevation_data.get_elevation( eachLonLat.lat,eachLonLat.lon)
                    if neighbourAltInMeters:
                        neighbourAlt = neighbourAltInMeters*3.28084
                        global maxStep
                        if  lowerAltitude <= neighbourAlt <= upperAltitude and math.fabs(currAlt-neighbourAlt)<maxStep:
                            cornerNeighbours.append(each)  
            else:
                continue
    return cornerNeighbours

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



def reconstruct_path(endPointID,needReverse,canvas):
    global startingPointAltitude,destinationPointAltitude,map_0,map_osm
    global canvas_1,UIZoomFactor,meetingPointID,flatModeOn
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
        while currentID in cameFrom.keys(): # this is never true, meeting point is not in cameFrom.keys()
            currentID = cameFrom[currentID]
            total_path.append(currentID)
        total_path.reverse()            
        total_path.remove(endPointID)   # endPoint is starting point
        for each in total_path:
            coords = pointIDToCoordPoint(each)
            if flatModeOn:
                canvas.create_oval([coords.x*UIZoomFactor-3,coords.y*UIZoomFactor-3,coords.x*UIZoomFactor+3,coords.y*UIZoomFactor+3],fill="green",outline = "green",activefill="green",activeoutline = "green", activewidth = 10)
            else:
                folium.Marker(location=[coordPointToLonLat(coords).lat, coordPointToLonLat(coords).lon]).add_to(map_osm)
                folium.Marker(location=[coordPointToLonLat(coords).lat, coordPointToLonLat(coords).lon]).add_to(map_0)
                canvas.create_oval([coords.x*UIZoomFactor-3,coords.y*UIZoomFactor-3,coords.x*UIZoomFactor+3,coords.y*UIZoomFactor+3],fill="red",outline = "red",activefill="red",activeoutline = "red", activewidth = 10)
        total_path.insert(0,endPointID)
    else:
        global cameFromE
        cameFrom = cameFromE
        while currentID in cameFrom.keys():
            currentID = cameFrom[currentID]
            coords = pointIDToCoordPoint(currentID)
            if flatModeOn:
                canvas.create_oval([coords.x*UIZoomFactor-3,coords.y*UIZoomFactor-3,coords.x*UIZoomFactor+3,coords.y*UIZoomFactor+3],fill="green",outline = "green",activefill="green",activeoutline = "green", activewidth = 10 )
            else:
                folium.Marker(location=[coordPointToLonLat(coords).lat, coordPointToLonLat(coords).lon]).add_to(map_osm)
                folium.Marker(location=[coordPointToLonLat(coords).lat, coordPointToLonLat(coords).lon]).add_to(map_0)
                canvas.create_oval([coords.x*UIZoomFactor-3,coords.y*UIZoomFactor-3,coords.x*UIZoomFactor+3,coords.y*UIZoomFactor+3],fill="red",outline = "red",activefill="red",activeoutline = "red", activewidth = 10 )

            total_path.append(currentID)

    return total_path



def aStar(startID,goalID,map_data,isStart):
    global flatModeOn,startingPointAltitude,destinationPointAltitude,lowerAltitude,upperAltitude
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
    global EARTH_RADIUS, xDimension,yDimension,map_0,map_osm

    centerLat = line[1]
    centerLon = line[2]
    radius = line[0]
    center = lonLat(centerLon,centerLat)
    # (x,y) = (lon * cos(lat avg) , lat)
    # d = R * sqrt((y2-y1)^2 + (x2-x1)^2)
    
    if boundaries.bottomLat > centerLat or boundaries.topLat < centerLat or boundaries.leftLon > centerLon or boundaries.rightLon < centerLon:
        return 0

    folium.CircleMarker(location=[centerLat, centerLon], radius=7000,popup=line[3], color='#fa0b0b',fill_color='#fa0b0b').add_to(map_0)
    folium.CircleMarker(location=[centerLat, centerLon], radius=7000,popup=line[3], color='#fa0b0b',fill_color='#fa0b0b').add_to(map_osm)
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
            checkLonlat = coordPointToLonLat(checkPoint)
            if x>=0 and x< xDimension and y>=0 and y<yDimension and haversine(checkLonlat,center) <= 9:
                updated_map_data[y][x] = 1

    np.savetxt("updated_data.csv", updated_map_data ,fmt='%d', delimiter=',') 
    
    return rectBoundariesInXY(leftX,rightX,topY,bottomY)



def writeToFile(file):
    global altitude,elevationArray,upperBoundArray,lowerBoundArray,route, startingPointAltitude,indexArray,lonArray,latArray
    indexArray = []            #exclude starting and ending point since elevation = 0
    lonArray = []
    latArray = []

    index = 0
    file.write(str(index)+"   "+str(startingPoint.lon)+"   "+str(startingPoint.lat)+"   "+str(elevation_data.get_elevation(startingPoint.lat,startingPoint.lon))+"\n")
    previousLonLat = startingPoint
    previousElevation = startingPointAltitude + altitude
    elevationArray = []
    upperBoundArray = []
    lowerBoundArray = []
    for each in route:
        coords = pointIDToCoordPoint(each)
        currlonlat = coordPointToLonLat(coords)

        numOfSubPoints = haversine(previousLonLat,currlonlat) / 0.1
        for i in range (1,int(numOfSubPoints)+1):
            newLonLat = lonLat((numOfSubPoints-i)/numOfSubPoints*previousLonLat.lon+i/numOfSubPoints*currlonlat.lon,(numOfSubPoints-i)/numOfSubPoints*previousLonLat.lat+i/numOfSubPoints*currlonlat.lat)
            index += 1

            #▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾ standard Operation ▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾                                    
            currAltitudeInMeters = elevation_data.get_elevation(newLonLat.lat,newLonLat.lon)
            if currAltitudeInMeters:
                currAltitude = currAltitudeInMeters*3.28084
                upperBound = currAltitude + 400
                lowerBound = currAltitude + 300
                upperBoundArray.append(upperBound)
                lowerBoundArray.append(lowerBound)
                currElevation = 0.0
                if previousElevation > upperBound:
                    currElevation = upperBound
                elif previousElevation < lowerBound:
                    currElevation = lowerBound
                else:
                    currElevation = previousElevation
                previousElevation = currElevation
                
                indexArray.append(index)
                lonArray.append(newLonLat.lon)
                latArray.append(newLonLat.lat)
                elevationArray.append(currElevation)
                file.write(str(index)+"   "+str(newLonLat.lon)+"   "+str(newLonLat.lat)+"   "+str(currElevation)+"\n")
            else:
                print("****************************"+str(newLonLat.lon)+", "+str(newLonLat.lat)+" no elevation data available")
            #▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴standard Operation▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴
        previousLonLat = currlonlat

    numOfWayPoints = len(elevationArray)
    xAxieArray = []
    for i in range (0,numOfWayPoints):
        xAxieArray.append(i)

    pyplot.plot(xAxieArray,elevationArray,label="elevation") 
    pyplot.plot(xAxieArray,upperBoundArray,label="400 bound") 
    pyplot.plot(xAxieArray,lowerBoundArray,label="300 bound") 
    pyplot.show(block=False)

    index += 1
    file.write(str(index)+"   "+str(destinationPoint.lon)+"   "+str(destinationPoint.lat)+"   "+str(elevation_data.get_elevation(destinationPoint.lat,destinationPoint.lon))+"\n")
    return 0



def find_path(distance,boundaries):
    global root,UIZoomFactor,canvas_1,multithread,notReachable,route,loadStart,errorMessage
    global k1, k2, b1, b2, xDimension, yDimension, startingPoint, destinationPoint
    # xf = k1 * xi + b1
    # yf = k2 * yi + b2
    global fileName
    file = open(fileName+".txt", "w")
    map_data = genfromtxt('updated_data.csv', delimiter=',')
                    
    startingID = coordPointToPointID(lonLatToCoordPoint(startingPoint))
    destID = coordPointToPointID(lonLatToCoordPoint(destinationPoint))
    startX = pointIDToCoordPoint(startingID).x
    startY = pointIDToCoordPoint(startingID).y
    destX = pointIDToCoordPoint(destID).x
    destY = pointIDToCoordPoint(destID).y
    loadEnd = time.time()

    computeStart = time.time()
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
        multithread = False
        thread1 = threading.Thread(target = aStar, args = (startingID,destID,map_data,True))
        thread3 = threading.Thread(target = multi_thread_monitor,args = ())
        thread1.start()
        thread3.start()
        thread1.join()
        thread3.join()

    computeEnd = time.time()
   
    if not notReachable:
        if multithread:
            route = reconstruct_path(startingID,True,canvas_1) + reconstruct_path(destID,False,canvas_1)
        else:
            route = reconstruct_path(startingID,True,canvas_1)
        writeToFile(file)
        totalDis = haversine(startingPoint,coordPointToLonLat(pointIDToCoordPoint(route[0])))
        for iter in range (0, len(route)-1):                                                                                                                # MINUS ONE IS IT NECESSARY?????????????????????????????????
            currCoordPoint = pointIDToCoordPoint(route[iter])
            nextCoordPoint = pointIDToCoordPoint(route[iter+1])
            canvas_1.create_line(UIZoomFactor*currCoordPoint.x,UIZoomFactor*currCoordPoint.y,UIZoomFactor*nextCoordPoint.x,UIZoomFactor*nextCoordPoint.y)
            totalDis = totalDis + haversine(coordPointToLonLat(currCoordPoint),coordPointToLonLat(nextCoordPoint))
        
        totalDis = totalDis + haversine(coordPointToLonLat(pointIDToCoordPoint(route[len(route)-1])),destinationPoint) 
        distText = Label(root, text=("Total Load Time = " + str(round((loadEnd - loadStart),2))+"\nTotal Computation Time = " + str(round((computeEnd - computeStart),2))+"\nTotal Distance  = "+str(round(totalDis,2))+" KM"))
        distText.grid(row = 0,column = 0,sticky=W+S)                   
        updated_map_data = map_data
    else:
        messagebox.showerror('Error',errorMessage)

    x_anchor_start = UIZoomFactor*lonLatToCoordPoint(startingPoint).x
    y_anchor_start = UIZoomFactor*lonLatToCoordPoint(startingPoint).y
    x_anchor_end = UIZoomFactor*lonLatToCoordPoint(destinationPoint).x
    y_anchor_end = UIZoomFactor*lonLatToCoordPoint(destinationPoint).y

    xyS = [x_anchor_start-18, y_anchor_start-18] 
    xyE = [x_anchor_end-18, y_anchor_end-18]
    global droneTakeOffPhoto,droneLandingPhoto
    droneTakeOffPhoto = PhotoImage(file="droneTakeOff.png")
    canvas_1.create_image(xyS, anchor=NW, image=droneTakeOffPhoto)
    droneLandingPhoto = PhotoImage(file="droneLanding.png")
    canvas_1.create_image(xyE, anchor=NW, image=droneLandingPhoto)


    file.close()
    pass



def submitButtonPressed(lonS,latS,lonE,latE,alt, _fileName):
    global flatModeOn,altitude
    global fileName,canvas_2,canvas_2Exist
    global startingPoint,destinationPoint
    pyplot.close()

    flatModeOn = False
    canvas_2.destroy()
    canvas_2Exist = False
    try:
        float(lonS)
        float(latS)
        float(lonE)
        float(latE)
    except ValueError:
        messagebox.showerror('Error','Format Error.')
        return 0
    if not _fileName:
        messagebox.showerror('Error','Enter file name. ')
        return 0
    else:
        fileName = _fileName
    precision = 2
    if not lonS or not latS or not latE or not alt:
        messagebox.showerror('Error','At least one of the boxes is empty.')
    elif math.fabs((decimal.Decimal(lonS)).as_tuple().exponent) < precision or math.fabs((decimal.Decimal(latS)).as_tuple().exponent) < precision or math.fabs((decimal.Decimal(lonE)).as_tuple().exponent) < precision or math.fabs((decimal.Decimal(latE)).as_tuple().exponent) < precision or math.fabs((decimal.Decimal(alt)).as_tuple().exponent) < precision:
        messagebox.showerror('Error','At least one of the boxes does not have enough decimal places.')
    elif float(lonS) < -140 or float(lonS) > -52 or float(lonE) < -140 or float(lonE) > -52 or float(latS) < 41 or float(latS) > 83 or float(latE) < 41 or float(latE) > 83:
        messagebox.showerror('Error','Point(s) out of Canada.')
    elif float(alt)<300 or float(alt)>400:
        messagebox.showerror('Error','Altitude needs to be within 300 to 400 ft.')
    elif math.fabs(float(lonS)-float(lonE))>2 or math.fabs(float(latE)-float(latS))>2:
        result = messagebox.askyesno('Warning','Search Range is too large. Do you confirm to proceed?')
        if result:
            startingPoint.lon = float(lonS)
            startingPoint.lat = float(latS)
            destinationPoint.lon = float(lonE)
            destinationPoint.lat = float(latE)
            altitude = float(alt)
            find_path_main()
    else:
        startingPoint.lon = float(lonS)
        startingPoint.lat = float(latS)
        destinationPoint.lon = float(lonE)
        destinationPoint.lat = float(latE)
        altitude = float(alt)
        find_path_main()



def initializeGlobalVariables():    
    global route,k1,k2,b1,b2,xDimension,yDimension,notReachable,errorMessage,closedSetE,closedSetS,startingPointAltitude,destinationPointAltitude
    global cameFromE,cameFromS,stop,multithread,found,meetingPointID,UIZoomFactor,canvas_1,root, fileName,standardFinished
    route = []      
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
    UIZoomFactor = 8
    canvas_1 = Canvas(root,width=1000,height = 800,background = "white")
    canvas_1.grid(row = 0,column = 0,columnspan = 4)
    startingPointAltitude = 0.0
    destinationPointAltitude = 0.0
    standardFinished = False

def find_path_main():
    global loadStart,map_0,map_osm
    loadStart = time.time()
    global flatModeOn,underFifteen
    # ▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾ MAP SETUP ▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾
    if not flatModeOn:
        initializeGlobalVariables()
        global standardFinished,destinationPointAltitude
        global k1, k2, b1, b2, xDimension, yDimension, root, UIZoomFactor, canvas_1
        global route, startingPoint, destinationPoint, notReachable, errorMessage,startingPointAltitude
        
        
        
        map_0 = folium.Map(location=[(startingPoint.lat+destinationPoint.lat)/2,(startingPoint.lon+destinationPoint.lon)/2], tiles='Stamen Terrain',
                   zoom_start=8)
        folium.LatLngPopup().add_to(map_0)        
        map_osm = folium.Map(location=[(startingPoint.lat+destinationPoint.lat)/2,(startingPoint.lon+destinationPoint.lon)/2],zoom_start=8)
        folium.LatLngPopup().add_to(map_osm)
        



        tempStartAlt = elevation_data.get_elevation(startingPoint.lat, startingPoint.lon)
        tempEndAlt = elevation_data.get_elevation(destinationPoint.lat, destinationPoint.lon)
        if tempStartAlt:
            startingPointAltitude = tempStartAlt * 3.28084
        else:
            messagebox.showerror('Error','No altitude data available at starting point.')
            return 0

        if tempEndAlt:
            destinationPointAltitude = tempEndAlt * 3.28084
        else:
            messagebox.showerror('Error','No altitude data available at destination point.')
            return 0

        distance = haversine(startingPoint,destinationPoint)
        if distance < 15:
            xDimension=200
            yDimension=200
            UIZoomFactor = 5
        else:
            xDimension = 150
            yDimension = 150
            UIZoomFactor = 6.5

        boundaries = get_boundaries(startingPoint.lon,startingPoint.lat,destinationPoint.lon,destinationPoint.lat)
        k1 = (boundaries.rightLon - boundaries.leftLon)/(xDimension)
        k2 = (boundaries.bottomLat - boundaries.topLat)/(yDimension)
        b1 = boundaries.leftLon
        b2 = boundaries.topLat

        map_data = genfromtxt('map_data.csv', delimiter=',')
        np.savetxt("updated_data.csv", map_data ,fmt='%d', delimiter=',') 
        geoDataFile = open("nfz.txt","r")
        rows = (row.strip().split() for row in geoDataFile)
        global geoData
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
            geoData[rowNum].append(name)
            xyText = [UIZoomFactor*lonToX(geoData[rowNum][2]),UIZoomFactor*latToY(geoData[rowNum][1])+20] 
            nameObject = canvas_1.create_text(xyText, text=name)
            nameTextObjects.append(nameObject)
            rowNum = rowNum + 1
            
        # check if starting/ending points are in the NFZ
        for eachLine in geoData:
            if eachLine != []:
                distFromStart = haversine(startingPoint,lonLat(eachLine[2],eachLine[1]))
                if distFromStart <= float(eachLine[0]):
                    notReachable = True
                    errorMessage = "starting not reachable"
                    break
                distFromEnd = haversine(destinationPoint,lonLat(eachLine[2],eachLine[1]))
                if distFromEnd <= float(eachLine[0]):
                    notReachable = True
                    errorMessage = "ending not reachable"
                    break
                rectXY = setUpMap(eachLine,map_data,boundaries)
                if rectXY != 0:
                    xy = [UIZoomFactor*rectXY.leftX, UIZoomFactor*rectXY.bottomY, UIZoomFactor*rectXY.rightX, UIZoomFactor*rectXY.topY] 
                    canvas_1.create_arc(xy, start=0, extent=359.999999999,outline = "grey90",fill = "grey90")
                    
        for each in nameTextObjects:
            canvas_1.tag_raise(each)
    else:
        distance = haversine(startingPoint,destinationPoint)
        boundaries = get_boundaries(startingPoint.lon,startingPoint.lat,destinationPoint.lon,destinationPoint.lat)
    #▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴ MAP SETUP ▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴

    underFifteen = True
    elevationArray = []
    upperBoundArray = []
    lowerBoundArray = []
    if notReachable:
        print(errorMessage)
        messagebox.showerror('Error',errorMessage)
        return 0
    if distance <= 15:
        global fileName
        file = open(fileName+".txt", "w")
        loadEnd = time.time()
        computeStart = time.time()

        index = 0
        file.write(str(index)+"   "+str(startingPoint.lon)+"   "+str(startingPoint.lat)+"   "+"0"+"\n")
        global altitude
        previousElevation = startingPointAltitude+altitude
        numOfSubPoints = haversine(startingPoint,destinationPoint) / 0.1
        straightline = True

        for i in range (1,int(numOfSubPoints)+1):                                                                                                               # PLUS ONE IS IT NECESSARY?????????????????????????????????
            if straightline:
                newLonLat = lonLat((numOfSubPoints-i)/numOfSubPoints*startingPoint.lon+i/numOfSubPoints*destinationPoint.lon,(numOfSubPoints-i)/numOfSubPoints*startingPoint.lat+i/numOfSubPoints*destinationPoint.lat)
                for each in geoData:
                    if each != []:
                        dist = haversine(newLonLat,lonLat(each[2],each[1]))
                        if dist <= float(each[0]):
                                print("not a straight line")
                                straightline = False
                                break
                if not straightline:
                    break
                index += 1
                currAltitude = elevation_data.get_elevation( newLonLat.lat,newLonLat.lon)*3.28084
                if currAltitude:
                    upperBound = currAltitude + 400
                    lowerBound = currAltitude + 300
                    upperBoundArray.append(upperBound)
                    lowerBoundArray.append(lowerBound)
                    currElevation = 0.0
                    if previousElevation > upperBound:
                        currElevation = upperBound
                    elif previousElevation < lowerBound:
                        currElevation = lowerBound
                    else:
                        currElevation = previousElevation
                    previousElevation = currElevation
                    elevationArray.append(currElevation)
                    file.write(str(index)+"   "+str(newLonLat.lon)+"   "+str(newLonLat.lat)+"   "+str(currElevation)+"\n")
                else:
                    print("****************************"+str(newLonLat.lon)+", "+str(newLonLat.lat)+" no elevation data available")
                    file.write(str(index)+"   "+str(newLonLat.lon)+"   "+str(newLonLat.lat)+"   NO ALTITUDE DATA AVAILABLE\n")
        if straightline:
                file.write(str(index)+"   "+str(destinationPoint.lon)+"   "+str(destinationPoint.lat)+"   "+"0"+"\n")
                computeEnd = time.time()
                #▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾ UI ▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾
                totalDis = haversine(startingPoint,destinationPoint)
                startCoordPoint = lonLatToCoordPoint(startingPoint)
                endCoordPoint = lonLatToCoordPoint(destinationPoint)
                canvas_1.create_line(UIZoomFactor*startCoordPoint.x,UIZoomFactor*startCoordPoint.y,UIZoomFactor*endCoordPoint.x,UIZoomFactor*endCoordPoint.y)
                canvas_1.create_oval([startCoordPoint.x*UIZoomFactor-3,startCoordPoint.y*UIZoomFactor-3,startCoordPoint.x*UIZoomFactor+3,startCoordPoint.y*UIZoomFactor+3],fill="green",outline = "green",activefill="green",activeoutline = "green", activewidth = 10)
                canvas_1.create_oval([endCoordPoint.x*UIZoomFactor-3,endCoordPoint.y*UIZoomFactor-3,endCoordPoint.x*UIZoomFactor+3,endCoordPoint.y*UIZoomFactor+3],fill="green",outline = "green",activefill="green",activeoutline = "green", activewidth = 10)
                distText = Label(root, text=("Total Load Time = " + str(round((loadEnd - loadStart),2))+"\nTotal Computation Time = " + str(round((computeEnd - computeStart),2))+"\nTotal Distance  = "+str(round(totalDis,2))+" KM"))
                distText.grid(row = 0,column = 0,sticky=W+S)  
                createMiniMap(boundaries)
                #▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴ UI ▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴
        else:                                                                                                                   # distance < 15 but not a straight line
            find_path(distance,boundaries)
            createMiniMap(boundaries)
            standardFinished = True
            underFifteen = False
        file.close()
    else:                                                                                                                           # distance >= 15
        find_path(distance,boundaries)
        createMiniMap(boundaries)
        standardFinished = True
        underFifteen = False
    standardFinished = True
    if underFifteen:
        numOfWayPoints = len(elevationArray)
        if numOfWayPoints == len(upperBoundArray) and numOfWayPoints == len(lowerBoundArray):
            for i in range (0,20):
                for iter in range(1,numOfWayPoints-1):
                    averageOfTwoNeighbours = (elevationArray[iter-1]+elevationArray[iter+1])/2
                    if lowerBoundArray[iter] > averageOfTwoNeighbours:
                        elevationArray[iter] = lowerBoundArray[iter]
                    elif averageOfTwoNeighbours > upperBoundArray[iter]:
                        elevationArray[iter] = upperBoundArray[iter]
                    else:
                        elevationArray[iter] = averageOfTwoNeighbours
        else:
            messagebox.showerror('Error',"index error")    
        xAxieArray = []
        for i in range (0,numOfWayPoints):
            xAxieArray.append(i)
        pyplot.plot(xAxieArray,elevationArray,label="elevation") 
        pyplot.plot(xAxieArray,upperBoundArray,label="400 bound") 
        pyplot.plot(xAxieArray,lowerBoundArray,label="300 bound") 
        pyplot.show(block=False)
    map_0.save('sthelens.html')
    map_osm.save('osm.html')




def createMiniMap(boundaries):
    global canvas_1,photo
    photo = PhotoImage(file="canada-lower.png")
    canvas_1.create_image(750,0, anchor=NW, image=photo)
    canvas_1.create_rectangle(boundaries.leftLon*2.5843+1112.382,boundaries.bottomLat*(-4.561)+383.561,boundaries.rightLon*2.5843+1116.382,boundaries.topLat*(-4.561)+379.561,outline = "red",width = 3)
    canvas_1.create_rectangle(750,3,1000,190)



startLonTbox = Entry(root,width = 30)
startLatTbox = Entry(root,width = 30)
destLonTbox = Entry(root,width = 30)
destLatTbox = Entry(root,width = 30)
fileNameTbox = Entry(root,width = 30)
altTbox = Entry(root,width = 30)
canvas_2 = Canvas()
canvas_2Exist = False
photo2 = PhotoImage()



def draw_zoomout():
    global route,canvas_2,UIZoomFactor,geoData,startingPoint,destinationPoint, EARTH_RADIUS, xDimension,yDimension
    for each in route:
        coords = pointIDToCoordPoint(each)
        xy = [(coords.x*UIZoomFactor)*0.5+250-3,coords.y*UIZoomFactor*0.5+200-3,coords.x*UIZoomFactor*0.5+250+3,coords.y*UIZoomFactor*0.5+200+3]
        canvas_2.create_oval(xy,fill="red",outline = "red",activefill="red",activeoutline = "red", activewidth = 10)
    
    boundaries = get_boundaries(startingPoint.lon,startingPoint.lat,destinationPoint.lon,destinationPoint.lat)
    boundaries.bottomLat = boundaries.bottomLat-2
    boundaries.topLat = boundaries.topLat+2
    boundaries.rightLon = boundaries.rightLon +2
    boundaries.leftLon =  boundaries.leftLon -2

    for eachLine in geoData:
        if eachLine != []:
            centerLat = eachLine[1]
            centerLon = eachLine[2]
            radius = eachLine[0]
            center = lonLat(centerLon,centerLat)

            if boundaries.bottomLat > centerLat or boundaries.topLat < centerLat or boundaries.leftLon > centerLon or boundaries.rightLon < centerLon:
                continue

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
            rectXY = rectBoundariesInXY(leftX,rightX,topY,bottomY)
            
            xy = [UIZoomFactor*rectXY.leftX*0.5+250, UIZoomFactor*rectXY.bottomY*0.5+200, UIZoomFactor*rectXY.rightX*0.5+250, UIZoomFactor*rectXY.topY*0.5+200] 

            canvas_2.create_arc(xy, start=0, extent=359.999999999,outline = "grey90",fill = "grey90")

            name = eachLine[3]
            xyText = [UIZoomFactor*rectXY.leftX*0.5+250, UIZoomFactor*rectXY.bottomY*0.5+200+5] 
            nameObject = canvas_2.create_text(xyText, text=name)
    x_anchor_start = UIZoomFactor*lonLatToCoordPoint(startingPoint).x*0.5+250
    y_anchor_start = UIZoomFactor*lonLatToCoordPoint(startingPoint).y*0.5+200
    x_anchor_end = UIZoomFactor*lonLatToCoordPoint(destinationPoint).x*0.5+250
    y_anchor_end = UIZoomFactor*lonLatToCoordPoint(destinationPoint).y*0.5+200


    xyS = [x_anchor_start-18, y_anchor_start-18] 
    xyE = [x_anchor_end-18, y_anchor_end-18]
    global droneTakeOffPhoto,droneLandingPhoto
    droneTakeOffPhoto = PhotoImage(file="droneTakeOff.png")
    canvas_2.create_image(xyS, anchor=NW, image=droneTakeOffPhoto)
    droneLandingPhoto = PhotoImage(file="droneLanding.png")
    canvas_2.create_image(xyE, anchor=NW, image=droneLandingPhoto)

    global photo2
    photo2 = PhotoImage(file="canada-lower.png")
    canvas_2.create_image(750,0, anchor=NW, image=photo2)                  
    canvas_2.create_rectangle(boundaries.leftLon*2.5843+1112.382,boundaries.bottomLat*(-4.561)+383.561,boundaries.rightLon*2.5843+1116.382,boundaries.topLat*(-4.561)+379.561,outline = "red",width = 3)
    canvas_2.create_rectangle(750,3,1000,190)



def showCanvas2():
    global canvas_2,canvas_2Exist, standardFinished
    if not standardFinished:
        return 0
    if not canvas_2Exist:
        canvas_2 = Canvas(width=1000,height = 800,background = "white")
        canvas_2.grid(row = 0,column = 0,columnspan = 4)
        draw_zoomout()
        canvas_2Exist = True



def removeCanvas2():
    global canvas_2,canvas_2Exist
    if not standardFinished:
        return 0
    if canvas_2Exist:
        canvas_2.destroy()
        canvas_2Exist = False



def computeflat(_maxStep):
    # ▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾ exception cases ▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾▾
    global underFifteen
    if underFifteen:
        return 0
    if not _maxStep:
        messagebox.showerror('Error'," set a coefficient ")
        return 0
    try:
        float(_maxStep)
    except ValueError:
        messagebox.showerror('Error','Format Error.')
        return 0
    global standardFinished
    if not standardFinished:
        return 0
     #▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴ exception cases ▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴▴
    global maxStep,flatModeOn,upperAltitude,lowerAltitude,destinationPointAltitude,startingPointAltitude,canvas_1,notReachable
    maxStep = float(_maxStep)
    flatModeOn = True
    global closedSetS, closedSetE,cameFromE,cameFromS
    global stop,meetingPointID,multithread,found
    closedSetS = []
    closedSetE = []
    cameFromS = {}
    cameFromE = {}
    stop = False
    notReachable = False
    meetingPointID = 0
    multithread = True
    found = False
    if math.fabs(startingPointAltitude-destinationPointAltitude)<=500:
        average = (startingPointAltitude+destinationPointAltitude)/2
        upperAltitude = average +250
        lowerAltitude = average -250
    else:
        upperAltitude = max(startingPointAltitude,destinationPointAltitude)
        lowerAltitude = min(startingPointAltitude,destinationPointAltitude)
    find_path_main()
 


def moreEco():
    global fileName
    file = open(fileName+".txt", "w")
    global underFifteen,indexArray,lonArray,latArray
    if underFifteen:
        return 0
    global elevationArray,upperBoundArray,lowerBoundArray
    numOfWayPoints = len(elevationArray)
    if numOfWayPoints != len(upperBoundArray) or numOfWayPoints != len(lowerBoundArray):
        messagebox.showerror('Error',"index error")
        return 0

    pyplot.close()
    for i in range (0,100):
        for iter in range(1,numOfWayPoints-1):
            averageOfTwoNeighbours = (elevationArray[iter-1]+elevationArray[iter+1])/2
            if lowerBoundArray[iter] > averageOfTwoNeighbours:
                elevationArray[iter] = lowerBoundArray[iter]
            elif averageOfTwoNeighbours > upperBoundArray[iter]:
                elevationArray[iter] = upperBoundArray[iter]
            else:
                elevationArray[iter] = averageOfTwoNeighbours

    
    xAxieArray = []
    for i in range (0,numOfWayPoints):
        xAxieArray.append(i)

    pyplot.plot(xAxieArray,elevationArray,label="elevation") 
    pyplot.plot(xAxieArray,upperBoundArray,label="400 bound") 
    pyplot.plot(xAxieArray,lowerBoundArray,label="300 bound") 
    pyplot.show(block=False)


    file.write("0   "+str(startingPoint.lon)+"   "+str(startingPoint.lat)+"   "+str(elevation_data.get_elevation(startingPoint.lat,startingPoint.lon))+"\n")
    for i in range(0,len(indexArray)):
        file.write(str(indexArray[i])+"   "+str(lonArray[i])+"   "+str(latArray[i])+"   "+str(elevationArray[i])+"\n")
    file.write(str(len(indexArray)+1)+"   "+str(destinationPoint.lon)+"   "+str(destinationPoint.lat)+"   "+str(elevation_data.get_elevation(destinationPoint.lat,destinationPoint.lon))+"\n")
    file.close()
    pass
    
    

def starting_page():
    global canvas_1, root, startLonTbox, startLatTbox, destLonTbox, destLatTbox,altTbox,fileNameTbox
    canvas_1.grid(row = 0,column = 0,columnspan = 4)
    photo = PhotoImage(file="Geomap-Canada.png")
    canvas_1.create_image(20,20, anchor=NW, image=photo)

    startLonLbl = Label(root, text="Starting Longtitude:",  height=2)
    startLonLbl.grid(row=1, column=0,sticky=W)
    startLonTbox.grid(row=1, column =1,sticky=W) 

    startLatLbl = Label(root, text="Starting Latitude:",  height=2)
    startLatLbl.grid(row=1, column=2,sticky=W)
    startLatTbox.grid(row =1, column =3,sticky=W)

    destLonLbl = Label(root, text="Destination Longtitude:",  height=2)
    destLonLbl.grid(row=2, column=0,sticky=W)
    destLonTbox.grid(row =2, column =1,sticky=W) 

    destLatLbl = Label(root, text="Destination Latitude:", height=2)
    destLatLbl.grid(row=2, column=2,sticky=W)
    destLatTbox.grid(row =2, column =3,sticky=W) 

    altLbl = Label(root, text="Altitude:", height=2)
    altLbl.grid(row=3, column=0,sticky=W)
    altTbox.grid(row =3, column =1,sticky=W)

    maxStepTbox = Entry(root,width = 30)
    maxStepTbox.grid(row =4, column =0,sticky=W)
    maxStepTbox.insert(0, '50.0')

    fileNameTbox = Entry(root,width = 30)
    fileNameTbox.grid(row =3, column =2,sticky=W)
    fileNameTbox.insert(0, 'DDCRoute')

    submitButton = Button(root, text ="Submit and Save to File", command = lambda:submitButtonPressed(startLonTbox.get(),startLatTbox.get(),destLonTbox.get(),destLatTbox.get(),altTbox.get(), fileNameTbox.get()))
    submitButton.grid(row = 3, column = 3,sticky = W)

    zoomOutButton = Button(root,text = " +       ",command = lambda:removeCanvas2())
    zoomOutButton.grid(row = 1,column=3,sticky=S+E)
    zoomInButton = Button(root,text = " - ",command = lambda:showCanvas2())
    zoomInButton.grid(row = 1,column=3,sticky=S+E)

    showflatButton = Button(root,text = " Show Flat ",command = lambda:computeflat(maxStepTbox.get()))
    showflatButton.grid(row = 4,column=1,sticky=W)

    moreEcoButton = Button(root,text = " More ECO ",command = lambda:moreEco())
    moreEcoButton.grid(row = 4,column=3,sticky=S+W)
    root.mainloop()



def func(event):
    global startLonTbox, startLatTbox, destLonTbox, destLatTbox,altTbox,fileNameTbox
    print("You hit return.")
    submitButtonPressed(startLonTbox.get(),startLatTbox.get(),destLonTbox.get(),destLatTbox.get(),altTbox.get(),fileNameTbox.get())



root.bind('<Return>', func)
starting_page()
