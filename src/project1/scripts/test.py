from collections import namedtuple

# Class version
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Namedtuple version
Point2 = namedtuple('Point', 'x y')



p1 = Point(1, 2)
p2 = Point2(3, 4)

p3 = [0, 0]
p3[0] = 5
print(p3)
