from numpy import genfromtxt
import numpy as np
np.set_printoptions(threshold=np.nan)

class coordPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y




def aStar(start,goal):



    pass







print("This line will be printed.")
map_data = genfromtxt('map_data.csv', delimiter=',')
startPoint = coordPoint(0,0)
goalPoint = coordPoint(99,99)

print(map_data)


aStar(3,2)



