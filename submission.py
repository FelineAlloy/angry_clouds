import sys

class Drone:
    def __init__(self, x, y, B, phi):
        self.x = x
        self.y = y
        self.B = B
        self.phi = phi
        
        self.bandwidth_used_in = dict()

    def b(self, t):
        """Return remaining bandwidth of this drone at second t
        (given the 10-slot periodic pattern with phase)."""
        s = (t + self.phi) % 10
        if s < 2 or s > 7:
            bw = 0.0
        elif s == 2 or s == 7:
            bw = self.B / 2.0
        else:
            bw = self.B

        used = self.bandwidth_used_in.get(t, 0.0)
        return max(0.0, bw - used)

def read_input(path=None):
    if path is None or path == "-":
        f = sys.stdin
        close_after = False
    else:
        f = open(path, "r", encoding="utf-8")
        close_after = True
    M, N, FN, T = map(int, f.readline().split())

    try :
        drones = []
        for _ in range(M * N):
            x, y, B, phi = f.readline().split()
            drones.append(Drone(int(x), int(y), float(B), int(phi)))

        flows = {}
        for _ in range(FN):
            parts = f.readline().split()
            f_id = int(parts[0])
            x, y, t_start, s, m1, n1, m2, n2 = map(int, parts[1:])
            flows[f_id] = {
                'id': f_id,
                'x': x,
                'y': y,
                't_start': t_start,
                's': s,
                'm1': m1,
                'n1': n1,
                'm2': m2,
                'n2': n2
            }

        return M, N, T, drones, flows
    
    finally :
        if close_after:
            f.close()

M, N, T, drones, flows = read_input()

for time in range(T) :
    # Stores all the flows that are active at time = T.
    active_flows = active_flows = [f for f in flows.values() if f['t_start'] <= time and f['s'] > 0]#[]

    # Iterate over all the active flows
    for flow in active_flows :
        # Skip flow if it has transmitted all its data already
        if flow['s'] == 0 : continue

        # If the flow had not previously partially transmitted its data through a drone then assign the flow a attribute
        # 'prev' which stores the previously used drone. 
        if 'prev' not in flow :
            flow['prev'] = None

        # This attribute will store the output logs for the flow.
        if 'hist' not in flow :
            flow['hist'] = []

        prev_drone = flow['prev']

        curr_drone = None
        if prev_drone:
            slot = (time + prev_drone.phi) % 10
            is_peak = 3 <= slot <= 6
            if is_peak and prev_drone.b(time) > 0:
                curr_drone = prev_drone
        if curr_drone is None:
            eligible_drones = [
                d for d in drones
                if d.x in range(flow['m1'], flow['m2']+1)
                and d.y in range(flow['n1'], flow['n2']+1)
                and d.b(time) > 0
            ]
            curr_drone = max(eligible_drones, key=lambda x: x.b(time)) if eligible_drones else None
        # Skip to next flow if no available drones.
        if not curr_drone : 
            continue

        # Compute the amount of data to transmit, bounded by drone bw and data to be transmitted.
        flow['prev'] = curr_drone
        transmitted = min(curr_drone.b(time), flow['s'])
        flow['s'] -= transmitted

        # Store the time and amount of data transmitted at said time.
        curr_drone.bandwidth_used_in[time] = curr_drone.bandwidth_used_in.get(time, 0.0) + transmitted

        # Log activity for output
        flow['hist'].append((time, curr_drone.x, curr_drone.y, float(transmitted)))

# === AFTER the time loop, ALWAYS emit every flow ===
out_lines = []
for flow in flows.values():
    # Ensure hist exists
    hist = flow.get('hist', [])
    # Header line: f p
    out_lines.append(f"{int(flow['id'])} {int(len(hist))}")
    # p lines: t x y z (z must be double)
    for (t, x, y, z) in hist:
        # enforce int32 for t, x, y and double for z
        out_lines.append(f"{int(t)} {int(x)} {int(y)} {z:.6f}")

sys.stdout.write("\n".join(out_lines))
sys.stdout.flush()