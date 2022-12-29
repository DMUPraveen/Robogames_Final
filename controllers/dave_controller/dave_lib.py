
class Dave:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.left_v = 0
        self.right_v = 0
    

    def set_velcoity(self,left_v,righ_v):
        self.left_v = left_v
        self.right_v = righ_v
    


def update_dave_pose(packet,dave:Dave):
    dave.orientation = packet["robotAngleDegrees"]
    dave.x = packet["robot"][0]
    dave.y = packet["robot"][1]