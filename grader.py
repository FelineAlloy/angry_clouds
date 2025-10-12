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


def read_output():
    with open(out_file_name) as f:
        output = {}
        while True:
            header = f.readline()
            if not header:
                break
            fid, p = map(int, header.split())
            records = []
            for _ in range(p):
                t, x, y, z = f.readline().split()
                records.append((int(t), int(x), int(y), float(z)))
            output[fid] = records
    return output


def hop_distance(x1, y1, x2, y2):
    # Manhattan distance (grid hops)
    return abs(x1 - x2) + abs(y1 - y2)


def score_flow(flow, records):
    if not records:
        return 0

    s_total = flow['s']
    s_transmitted = sum(z for _, _, _, z in records)

    # 1. Total U2G Traffic Score
    total_traffic_score = s_transmitted / s_total

    # 2. Traffic Delay Score
    delay_score = 0
    Tdelay = 10
    for t, _, _, z in records:
        delay_score += (Tdelay / (t + Tdelay - flow['t_start'])) * (z / s_total)

    # 3. Transmission Distance Score
    dist_score = 0
    alpha = 0.1
    for t, x, y, z in records:
        hops = hop_distance(flow['x'], flow['y'], x, y)
        dist_score += (z / s_total) * (2 ** (-alpha * hops))

    # 4. Landing UAV Point Score
    landing_points = [(x, y) for _, x, y, _ in records]
    unique_points = []
    for p in landing_points:
        if not unique_points or unique_points[-1] != p:
            unique_points.append(p)
    k = len(unique_points)
    landing_score = 1 / k

    # Combine
    flow_score = 100 * (
        0.4 * total_traffic_score +
        0.2 * delay_score +
        0.3 * dist_score +
        0.1 * landing_score
    )

    print(f"Flow {flow['id']} score breakdown:")
    print(f"  Total U2G Traffic Score: {total_traffic_score:.4f}")
    print(f"  Traffic Delay Score:     {delay_score:.4f}")
    print(f"  Distance Score:          {dist_score:.4f}")
    print(f"  Landing Point Score:     {landing_score:.4f}")
    print(f"  => Final Flow Score:     {flow_score:.3f}\n")

    return flow_score, s_transmitted


def main():
    M, N, T, drones, flows = read_input()
    output = read_output()

    total_weighted = 0
    total_size = 0
    for flow in flows:
        fid = flow['id']
        records = output.get(fid, [])
        fscore, sent = score_flow(flow, records)
        total_weighted += fscore * flow['s']
        total_size += flow['s']

    total_score = total_weighted / total_size if total_size else 0
    print(f"Total Score: {total_score:.3f}")


if __name__ == "__main__":
    main()
