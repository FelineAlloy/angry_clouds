from file_utils import read_input

def solve() :
    in_file_name = "in1.in"
    out_file_name = "out.out"

    out_file = open(out_file_name, 'w')

    M, N, T, drones, flows = read_input(in_file_name)

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

            # Stick to prev drone if it exists and is at peak bw to minimise switch cost. 
            if prev_drone :
                slot = (time + prev_drone.phi) % 10
                is_peak = 3 <= slot <= 6
                if is_peak and prev_drone.b(time) > 0 : #prev_drone.b(time) == prev_drone.B
                    curr_drone = prev_drone

            else :
                # Build array of eleigible drones and pick the one with max bw to offer (GREEDY APPROACH).
                eligible_drones = [d for d in drones if d.x in range(flow['m1'], flow['m2']+1) and d.y in range(flow['n1'], flow['n2']+1) and d.b(time)>0]
                curr_drone = max(eligible_drones, key = lambda x: x.b(time)) if eligible_drones else None

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
            flow['hist'].append(f'{time} {curr_drone.x} {curr_drone.y} {transmitted}')

            # If all flow data is transmitted then write to output
            if flow['s'] == 0 :
                out_file.write(f"{flow['id']} {len(flow['hist'])}\n")
                for h in flow['hist'] :
                    out_file.write(f'{h}\n')

    out_file.close()

def main() :
    solve()

if __name__ == "__main__" :
    main()