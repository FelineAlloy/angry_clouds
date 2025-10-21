#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

// A struct to hold the details of a single transmission event for logging.
struct TransmissionLog {
    int time;
    int x;
    int y;
    double transmitted_data;
};

// Represents a single drone with its properties and bandwidth state.
struct Drone {
    int x, y;
    double B;
    int phi;

    // A map to store the bandwidth used at specific time points.
    // Key: time, Value: used bandwidth
    std::map<int, double> bandwidth_used_in;

    /**
     * @brief Calculates the remaining bandwidth of this drone at a given time.
     * The calculation is based on a 10-slot periodic pattern with a phase shift.
     * @param t The current time step.
     * @return The available bandwidth at time t.
     */
    double get_remaining_bandwidth(int t) const {
        int s = (t + phi) % 10;
        double base_bw = 0.0;

        // Bandwidth is only available in slots 2 through 7.
        if (s >= 2 && s <= 7) {
            if (s == 2 || s == 7) {
                base_bw = B / 2.0; // Half bandwidth at the edges of the window.
            } else {
                base_bw = B; // Full bandwidth in the peak window.
            }
        }

        double used_bw = 0.0;
        auto it = bandwidth_used_in.find(t);
        if (it != bandwidth_used_in.end()) {
            used_bw = it->second;
        }

        return std::max(0.0, base_bw - used_bw);
    }
};

// Represents a data flow with its requirements and transmission history.
struct Flow {
    int id;
    int x, y;
    int t_start;
    double s; // Remaining data to be sent.
    int m1, n1, m2, n2; // Geographic area for drone selection.

    // State variables for the simulation
    Drone* prev_drone = nullptr; // Pointer to the last used drone.
    std::vector<TransmissionLog> history; // Log of all transmissions for this flow.
};

int main() {
    // Optimize C++ standard streams for faster input/output.
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    int M, N, FN, T;
    std::cin >> M >> N >> FN >> T;

    std::vector<Drone> drones;
    drones.reserve(M * N);
    for (int i = 0; i < M * N; ++i) {
        int x, y, phi;
        double B;
        std::cin >> x >> y >> B >> phi;
        drones.push_back({x, y, B, phi, {}});
    }

    std::vector<Flow> flows;
    flows.reserve(FN);
    for (int i = 0; i < FN; ++i) {
        int f_id, x, y, t_start, m1, n1, m2, n2;
        double s;
        std::cin >> f_id >> x >> y >> t_start >> s >> m1 >> n1 >> m2 >> n2;
        flows.push_back({f_id, x, y, t_start, s, m1, n1, m2, n2, nullptr, {}});
    }

    // --- Main Simulation Loop ---
    for (int time = 0; time < T; ++time) {
        // Iterate over all flows to process active ones.
        for (Flow& flow : flows) {
            // A flow is active if it has started and still has data to send.
            if (flow.t_start <= time && flow.s > 0) {
                Drone* current_drone = nullptr;
                Drone* prev_drone = flow.prev_drone;

                // --- Drone Selection Logic ---
                // 1. Prioritize sticking with the previous drone if it's in a peak slot.
                if (prev_drone) {
                    int slot = (time + prev_drone->phi) % 10;
                    bool is_peak = (slot >= 3 && slot <= 6);
                    if (is_peak && prev_drone->get_remaining_bandwidth(time) > 0) {
                        current_drone = prev_drone;
                    }
                }

                // 2. If no drone yet, find the best available one.
                if (!current_drone) {
                    Drone* best_drone = nullptr;
                    double max_bw = -1.0;

                    // Find all eligible drones and select the one with the most bandwidth.
                    for (Drone& d : drones) {
                        if (d.x >= flow.m1 && d.x <= flow.m2 &&
                            d.y >= flow.n1 && d.y <= flow.n2) {
                            
                            double bw = d.get_remaining_bandwidth(time);
                            if (bw > max_bw) {
                                max_bw = bw;
                                best_drone = &d;
                            }
                        }
                    }
                    current_drone = best_drone;
                }

                // If a suitable drone was found, perform the transmission.
                if (current_drone) {
                    double available_bw = current_drone->get_remaining_bandwidth(time);
                    double amount_to_transmit = std::min(available_bw, flow.s);

                    if (amount_to_transmit > 0) {
                        flow.s -= amount_to_transmit;
                        current_drone->bandwidth_used_in[time] += amount_to_transmit;
                        flow.prev_drone = current_drone;
                        flow.history.push_back({time, current_drone->x, current_drone->y, amount_to_transmit});
                    }
                }
            }
        }
    }

    // --- Output Generation ---
    std::stringstream ss;
    // Set floating point precision for the entire output stream.
    ss << std::fixed << std::setprecision(6);

    for (const auto& flow : flows) {
        ss << flow.id << " " << flow.history.size() << "\n";
        for (const auto& log : flow.history) {
            ss << log.time << " " << log.x << " " << log.y << " " << log.transmitted_data << "\n";
        }
    }

    // Print the entire result at once.
    std::cout << ss.str();

    return 0;
}
