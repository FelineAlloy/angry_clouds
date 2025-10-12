import math

in_file_name = "in.txt"
out_file_name = "out.txt"

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
            d = 0
        elif t == 2 or t == 7:
            d = self.B / 2
        else:
            d = self.B

        if d > 0 and t in self.bandwidth_used_in :
            return d - self.bandwidth_used_in[t]
        
        return d


def read_input():
    with open(in_file_name) as f:
        M, N, FN, T = map(int, f.readline().split())

        # drones = [[None]*M for _ in range(N)]
        drones = []
        for _ in range(M * N):
            x, y, B, phi = f.readline().split()
            # drones[int(x)][int(y)] = Drone(int(x), int(y), float(B), int(phi))
            drones.append(Drone(int(x), int(y), float(B), int(phi)))

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
        if flow['t_start'] <= time :
            active_flows.append(flow)
            flows.remove(flow)

    for flow in active_flows :
        print(flow, time)
        if flow['s'] == 0 : continue
        if 'prev' not in flow :
            flow['prev'] = None

        if 'hist' not in flow :
            flow['hist'] = []

        prev_drone = flow['prev']

        if prev_drone and prev_drone.b(time) == prev_drone.B :
            curr_drone = prev_drone

        else :
            eligible_drones = [d for d in drones if d.x in range(flow['m1'], flow['n1']+1) and d.y in range(flow['m2'], flow['n2']+1) and d.b(time)>0]
            curr_drone = max(eligible_drones, key = lambda x: x.b(time))

        flow['prev'] = curr_drone
        transmitted = min(curr_drone.b(time), flow['s'])
        flow['s'] -= transmitted
        curr_drone.bandwidth_used_in[time] = transmitted

        flow['hist'].append(f'{time} {curr_drone.x} {curr_drone.y} {transmitted}')
        if flow['s'] == 0 :
            with open(out_file_name, 'a') as f :
                s = f'{flow['id']} {len(flow['hist'])}\n'
                f.write(s)
                for h in flow['hist'] :
                    f.write(f'{h}\n')