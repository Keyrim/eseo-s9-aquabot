class Target:
    def __init__(self):
        self.x = 0 # Reference position x
        self.y = 0 # Reference position y
    
    def updateAll(self, x, y, theta):
        self.x = x
        self.y = y

    def updateFromMsg(self, msg):
        self.x = msg.x[0]
        self.y = msg.y[0]