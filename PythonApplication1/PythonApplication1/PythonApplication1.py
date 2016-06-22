from numpy import genfromtxt

class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y



def aStar(start,goal):
    closedSet = []
    openSet = []
    openSet.append(start)
    cameFrom = []




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

aStar(coordPointToPointID(startPoint),coordPointToPointID(goalPoint))

# print(map_data[startPoint.x][startPoint.y])



