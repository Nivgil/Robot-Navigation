class Position(object):

    def __init__(self, x=None, y=None, dx=None, dy=None):
        self.dx = dx
        self.dy = dy
        self.x = x
        self.y = y

    def set_position(self, x=None, y=None, dx=None, dy=None):
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if dx is not None:
            self.dx = dx
        if dy is not None:
            self.dy = dy

    def __str__(self):
        string = 'x-[{}], y-[{}], dx-[{}], dy-[{}]'.format(self.x, self.y, self.dx, self.dy)
        return string

    def __repr__(self):
        string = 'x-[{}], y-[{}], dx-[{}], dy-[{}]'.format(self.x, self.y, self.dx, self.dy)
        return string
