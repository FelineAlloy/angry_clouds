import math

in_file_name = "in.in"
out_file_name = "out.out"

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

def read_input():
    with open(in_file_name) as f:
        M, N, FN, T = map(int, f.readline().split())

        drones = [[None]*M for _ in range(N)]
        for _ in range(M * N):
            x, y, B, phi = f.readline().split()
            drones[int(x)][int(y)] = Drone(int(x), int(y), float(B), int(phi))

        flows = []
        for _ in range(FN):
            parts = f.readline().split()
            f_id = int(parts[0])
            x, y, t_start, s, m1, n1, m2, n2 = map(int, parts[1:])
            flows.append({
                'id': f_id,
                'x': x,
                'y': y,
                't_start': t_start,
                's': s,
                'm1': m1,
                'n1': n1,
                'm2': m2,
                'n2': n2
            })
    return M, N, T, drones, flows

M, N, T, drones, flows = read_input()

active_flows = []
for time in range(T) :
    for flow in flows :
        if flow['t_start'] >= time :
            active_flows.append(flow)
            flows.remove(flow)

    for flow in active_flows :
        if 'prev' not in flow :
            flow['prev'] = None

        p_x, p_y = flow['prev']
        prev_drone = drones[p_y][p_x]

        if prev_drone.b(T) == prev_drone.B :
            curr_drone = prev_drone

        else :
            eligible_drones = [drones[y][x] for y in range(flow['m2'], flow['n2'] + 1) for x in range(flow['m1'], flow['n1'] + 1) if drones[y][x].b(T) > 0]
            curr_drone = eligible_drones.max(key = lambda x: x.b(T))

        

            



