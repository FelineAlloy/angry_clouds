class Drone:
    def __init__(self, x, y, B, phi):
        self.x = x
        self.y = y
        self.B = B
        self.phi = phi

    def b(self, t):
        t = (t + self.phi) % 10
        if t < 2 or t > 7:
            return 0
        elif t == 2 or t == 7:
            return self.B / 2
        else:
            return self.B