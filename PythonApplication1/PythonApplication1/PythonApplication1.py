from numpy import genfromtxt

class coordPoint:
    def __init__(self, **kwargs):
        return super().__init__(**kwargs)


def aStar(start,goal):
    print(start+goal)
    pass

















print("This line will be printed.")
map_data = genfromtxt('map_data.csv', delimiter=',')

print(map_data[99][99])
aStar(3,2)



