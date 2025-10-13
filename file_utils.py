from drone import Drone

def read_input(in_file_name):
    with open(in_file_name) as f:
        M, N, FN, T = map(int, f.readline().split())

        drones = [[None]*M for _ in range(N)]
        # drones = []
        for _ in range(M * N):
            x, y, B, phi = f.readline().split()
            drones[int(x)][int(y)] = Drone(int(x), int(y), float(B), int(phi))
            # drones.append(Drone(int(x), int(y), float(B), int(phi)))
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

def read_output(out_file_name):
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