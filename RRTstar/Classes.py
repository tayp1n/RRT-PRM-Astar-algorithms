from shapely.geometry import Point, Polygon, LineString


class Figures:

    def __init__(self, coord_list):
        self.coord_list = coord_list
        self.polygon = Polygon(coord_list)


class Nodes:

    def __init__(self, x_coord, y_coord, neighbour_indexes, parent_index, cost, self_index):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.neighbour_indexes = neighbour_indexes
        self.parent_index = parent_index
        self.cost = cost
        self.self_index = self_index
