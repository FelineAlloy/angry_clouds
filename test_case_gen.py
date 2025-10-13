import random
from test import M, N, FN, T, MAX_B, MAX_Q_TOTAL



OUT_FILE = "in1.in"

def generate_drones(M, N):
    """Generate UAVs with random bandwidth and phase."""
    drones = []
    for x in range(M):
        for y in range(N):
            B = round(random.uniform(1.0, MAX_B), 2)
            phi = random.randint(0, 9)
            drones.append((x, y, B, phi))
    return drones

def generate_flows(FN, M, N, T):
    """Generate random flows based on grid and time limits."""
    flows = []
    for f in range(1, FN + 1):
        # Access UAV
        x = random.randint(0, M - 1)
        y = random.randint(0, N - 1)

        # Flow parameters
        t_start = random.randint(0, max(0, T - 1))
        s = random.randint(1, MAX_Q_TOTAL)

        # Random landing rectangle
        m1 = random.randint(0, M - 1)
        m2 = random.randint(m1, M - 1)
        n1 = random.randint(0, N - 1)
        n2 = random.randint(n1, N - 1)

        flows.append((f, x, y, t_start, s, m1, n1, m2, n2))
    return flows

def write_input_file(M, N, FN, T, drones, flows, filename=OUT_FILE):
    """Write input file according to Huawei format."""
    with open(filename, "w") as f:
        f.write(f"{M} {N} {FN} {T}\n")
        for (x, y, B, phi) in drones:
            f.write(f"{x} {y} {B:.2f} {phi}\n")
        for flow in flows:
            f.write(" ".join(map(str, flow)) + "\n")

def main():
    drones = generate_drones(M, N)
    flows = generate_flows(FN, M, N, T)
    write_input_file(M, N, FN, T, drones, flows)
    # print(f"✅ Random test case generated: '{OUT_FILE}'")
    # print(f"  Grid: {M}×{N}")
    # print(f"  Flows: {FN}")
    # print(f"  Time duration: {T}s")
    # print(f"  Max B: {MAX_B} Mbps, Max Flow Size: {MAX_Q_TOTAL} Mbits")

if __name__ == "__main__":
    main()
