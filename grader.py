from drone import Drone
from file_utils import read_input, read_output

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
    in_file_name = "in1.in"
    out_file_name = "out.out"
    M, N, T, drones, flows = read_input(in_file_name)
    output = read_output(out_file_name)

    total_weighted = 0
    total_size = 0
    for fid, records in output.items():
        flow = flows.get(fid, [])
        fscore, sent = score_flow(flow, records)
        total_weighted += fscore * flow['s']
        total_size += flow['s']

    total_score = total_weighted / total_size if total_size else 0
    print(f"Total Score: {total_score:.3f}")


if __name__ == "__main__":
    main()
