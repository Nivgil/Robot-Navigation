class Position(object):

    def __init__(self, x, y, dx=0, dy=1):
        if dx == 0 and dy == 0:
            self._dx = 0
            self._dy = 1
        else:
            self._dx = dx
            self._dy = dy
        self._x = x
        self._y = y

    def get_coordinates(self):
        return self._x, self._y

    def get_pose(self):
        return self._dx, self._dy

    def set_coordinates(self, x, y):
        self._x = x
        self._y = y

    def set_pose(self, dx, dy):
        if dx == 0 and dy == 0:
            self._dx = 0
            self._dy = 1
        else:
            self._dx = dx
            self._dy = dy

    def set_position(self, x, y, dx, dy):
        self.set_coordinates(x, y)
        self.set_pose(dx, dy)
