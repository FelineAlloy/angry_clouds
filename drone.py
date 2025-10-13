class Drone:
    def __init__(self, x, y, B, phi):
        self.x = x
        self.y = y
        self.B = B
        self.phi = phi
        self.bandwidth_used_in = dict()

    def b(self, t):
        t = (t + self.phi) % 10
        if t < 2 or t > 7:
            bw = 0
        elif t == 2 or t == 7:
            bw = self.B / 2
        else:
            bw = self.B

        return bw - self.bandwidth_used_in[t] if t in self.bandwidth_used_in else bw