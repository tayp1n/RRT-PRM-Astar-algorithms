# http://bit.do/fVTi2
from shapely.geometry import Polygon


# class for Figures
class Figures:

    def __init__(self, coord_list):
        self.coord_list = coord_list
        self.polygon = Polygon(coord_list)


# class for nodes
class Nodes:

    def __init__(self, x_coord, y_coord, parent_index, cost):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.parent_index = parent_index
        self.cost = cost
