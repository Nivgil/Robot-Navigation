class Obstacles:

    def __init__(self, right=-1.0, center=-1.0, left=-1.0):
        self.right = right
        self.center = center
        self.left = left

    def set_obstacles(self, right=-1.0, center=-1.0, left=-1.0):
        self.right = right
        self.center = center
        self.left = left

    def __str__(self):
        string = 'r_obstacle-[{}], c_obstacle-[{}], left_obstacle-[{}]'.format(self.right, self.center, self.left)
        return string

    def __repr__(self):
        string = 'r_obstacle-[{}], c_obstacle-[{}], left_obstacle-[{}]'.format(self.right, self.center, self.left)
        return string

