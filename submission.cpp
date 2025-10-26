// #include <algorithm>
// #include <cmath>
// #include <iomanip>
// #include <iostream>
// #include <limits>
// #include <sstream>
// #include <string>
// #include <utility>
// #include <vector>
// #include <climits>
// using namespace std;

// struct UAV {
//     int x, y;
//     int phi;
//     double B;
//     // Bandwidth pattern for one 10-second cycle (indices 0-9):
//     double pattern[10];
//     // Prefix sum of bandwidth from time 0 up to T (length T+1, filled later)
//     vector<double> prefix;
// };

// struct Flow {
//     int id;
//     int sx, sy;        // source UAV coordinates
//     int start_time;
//     double volume;     // total Mbits remaining (initially = s)
//     // Last scheduled landing UAV (for potential use if we wanted stickiness)
//     // Not strictly needed in this greedy approach, but kept for clarity:
//     int last_landing_idx;
//     // Output records for this flow:
//     struct Record { int t, x, y; double z; };
//     vector<Record> schedule;
//     // Candidate landing UAV list (sorted by our metric)
//     struct Candidate { int uav_idx; double metric; };
//     vector<Candidate> candidates;
// };

// int main() {
//     ios::sync_with_stdio(false);
//     cin.tie(NULL);

//     int M, N, FN, T;
//     if(!(cin >> M >> N >> FN >> T)) {
//         return 0;
//     }
//     int totalUAV = M * N;
//     vector<UAV> drones;
//     drones.reserve(totalUAV);
//     // Map from grid coordinate to UAV index in drones vector
//     vector<vector<int>> uavIndex(M, vector<int>(N, -1));

//     // Precompute hop penalty for distances up to M+N (max Manhattan distance)
//     int maxDist = M + N;
//     vector<double> hopPenalty(maxDist + 1);
//     for(int d = 0; d <= maxDist; ++d) {
//         hopPenalty[d] = pow(2.0, -0.1 * d);
//     }

//     // Read UAV data
//     for(int i = 0; i < totalUAV; ++i) {
//         int x, y, phi;
//         double B;
//         cin >> x >> y >> B >> phi;
//         UAV u;
//         u.x = x;
//         u.y = y;
//         u.phi = phi;
//         u.B = B;
//         // Compute base pattern (period = 10)
//         // Slots: 0,1,8,9 -> 0 Mbps; 2,7 -> B/2; 3-6 -> B
//         for(int s = 0; s < 10; ++s) {
//             if(s == 2 || s == 7) {
//                 u.pattern[s] = B * 0.5;
//             } else if(s >= 3 && s <= 6) {
//                 u.pattern[s] = B;
//             } else {
//                 u.pattern[s] = 0.0;
//             }
//         }
//         // Initialize prefix array of length T+1
//         u.prefix.assign(T+1, 0.0);
//         // Compute prefix sums of actual bandwidth from time 0 to T-1
//         double cum = 0.0;
//         for(int t = 0; t < T; ++t) {
//             // capacity at time t = pattern[(phi + t) % 10]
//             double bw = u.pattern[(u.phi + t) % 10];
//             cum += bw;
//             u.prefix[t+1] = cum;
//         }
//         drones.push_back(move(u));
//         uavIndex[x][y] = i;
//     }

//     vector<Flow> flows;
//     flows.reserve(FN);

//     // Read flow data
//     for(int i = 0; i < FN; ++i) {
//         int f, sx, sy, tstart, m1, n1, m2, n2;
//         double s;
//         cin >> f >> sx >> sy >> tstart >> s >> m1 >> n1 >> m2 >> n2;
//         Flow flow;
//         flow.id = f;
//         flow.sx = sx;
//         flow.sy = sy;
//         flow.start_time = tstart;
//         flow.volume = s;
//         flow.last_landing_idx = -1;
//         // Build candidate list of UAVs in [m1..m2]x[n1..n2]
//         // and compute metric = total capacity from tstart * distance factor
//         flow.candidates.reserve((m2 - m1 + 1) * (n2 - n1 + 1));
//         // Clamp region to grid bounds (just in case)
//         if(m1 < 0) m1 = 0;
//         if(m2 >= M) m2 = M - 1;
//         if(n1 < 0) n1 = 0;
//         if(n2 >= N) n2 = N - 1;
//         for(int X = m1; X <= m2; ++X) {
//             for(int Y = n1; Y <= n2; ++Y) {
//                 int uavIdx = uavIndex[X][Y];
//                 if(uavIdx < 0) continue; // just a safety check
//                 // Manhattan distance from source (sx,sy) to this UAV (X,Y)
//                 int dist = abs(flow.sx - X) + abs(flow.sy - Y);
//                 // Total capacity from flow.start_time to T on this UAV:
//                 double totalCap = drones[uavIdx].prefix[T] - drones[uavIdx].prefix[flow.start_time];
//                 if(totalCap < 1e-9) {
//                     // If this UAV has zero capacity in the flow's remaining time, skip
//                     continue;
//                 }
//                 double metric = totalCap * hopPenalty[dist];
//                 flow.candidates.push_back({uavIdx, metric});
//             }
//         }
//         // Sort candidates by descending metric (highest first)
//         sort(flow.candidates.begin(), flow.candidates.end(), [&](const Flow::Candidate &a, const Flow::Candidate &b){
//             return a.metric > b.metric;
//         });
//         flows.push_back(move(flow));
//     }

//     // Sort flows by ID for output stability (though input might already be sorted by f)
//     sort(flows.begin(), flows.end(), [&](const Flow &a, const Flow &b){
//         return a.id < b.id;
//     });

//     // Prepare active flow list and capacity tracking
//     vector<int> activeFlows;
//     activeFlows.reserve(FN);
//     // We will manage remaining capacity per UAV per time slot in an array
//     vector<double> capLeft; capLeft.resize(drones.size());

//     // Simulation from t=0 to T-1
//     for(int t = 0; t < T; ++t) {
//         // Build list of active (not finished, started) flows at time t
//         activeFlows.clear();
//         for(size_t j = 0; j < flows.size(); ++j) {
//             if(flows[j].volume > 1e-9 && flows[j].start_time <= t) {
//                 activeFlows.push_back((int)j);
//             }
//         }
//         if(activeFlows.empty()) {
//             continue; // no active flows at this time
//         }
//         // Sort active flows by remaining volume (descending)
//         sort(activeFlows.begin(), activeFlows.end(), [&](int a, int b){
//             if(fabs(flows[a].volume - flows[b].volume) < 1e-9) {
//                 // tie-break: earlier start or lower ID
//                 if(flows[a].start_time != flows[b].start_time)
//                     return flows[a].start_time < flows[b].start_time;
//                 return flows[a].id < flows[b].id;
//             }
//             return flows[a].volume > flows[b].volume;
//         });

//         // Initialize remaining capacity for each UAV at time t
//         for(size_t u = 0; u < drones.size(); ++u) {
//             // base bandwidth at time t given UAV’s pattern and phase
//             capLeft[u] = drones[u].pattern[(drones[u].phi + t) % 10];
//         }

//         // Allocate capacity to flows in priority order
//         for(int idx : activeFlows) {
//             Flow &flow = flows[idx];
//             if(flow.volume < 1e-9) continue; // already finished by earlier allocation
//             // Determine which UAV (from flow.candidates) to use at this time
//             for(const Flow::Candidate &cand : flow.candidates) {
//                 int uavIdx = cand.uav_idx;
//                 // Only consider if UAV has capacity available and is within distance (cand list ensures region)
//                 if(capLeft[uavIdx] <= 1e-12) {
//                     continue; // no bandwidth left here this second
//                 }
//                 // Determine allocation amount = min(flow remaining, UAV remaining capacity)
//                 double send = flow.volume;
//                 if(send > capLeft[uavIdx]) {
//                     send = capLeft[uavIdx];
//                 }
//                 if(send <= 1e-12) {
//                     continue;
//                 }
//                 // Assign this transmission
//                 flow.schedule.push_back({t, drones[uavIdx].x, drones[uavIdx].y, send});
//                 flow.volume -= send;
//                 capLeft[uavIdx] -= send;
//                 flow.last_landing_idx = uavIdx;
//                 // Flow can use only one UAV per time, so break after allocation
//                 break;
//             }
//             // If no UAV had capacity (flow couldn't send this second), we do nothing – it remains for future.
//         }
//     }

//     // Output the schedule for each flow
//     // Already sorted by flow.id ascending
//     cout.setf(std::ios::fixed);
//     cout<<setprecision(6);
//     for(const Flow &flow : flows) {
//         // Number of records for this flow
//         cout << flow.id << " " << flow.schedule.size() << "\n";
//         for(const auto &rec : flow.schedule) {
//             cout << rec.t << " " << rec.x << " " << rec.y << " " << rec.z << "\n";
//         }
//     }

//     return 0;
// }

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <climits>
using namespace std;

struct UAV {
    int x, y;
    int phi;
    double B;
    // Bandwidth pattern for one 10-second cycle (indices 0-9):
    double pattern[10];
    // Prefix sum of bandwidth from time 0 up to T (length T+1, filled later)
    vector<double> prefix;
};

struct Flow {
    int id;
    int sx, sy;        // source UAV coordinates
    int start_time;
    double volume;     // remaining Mbits
    double initial_volume; // keep original to track progress
    int last_landing_idx;  // last chosen landing UAV (index in drones), -1 if none
    struct Record { int t, x, y; double z; };
    vector<Record> schedule;

    struct Candidate {
        int uav_idx;
        int dist;           // Manhattan distance
        double hopFactor;   // 2^(-0.1*dist)
        double totalCap;    // total capacity from start_time..T (for tie-breaks)
    };
    vector<Candidate> candidates;
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(NULL);

    int M, N, FN, T;
    if(!(cin >> M >> N >> FN >> T)) return 0;

    const int totalUAV = M * N;
    vector<UAV> drones;
    drones.reserve(totalUAV);

    vector<vector<int>> uavIndex(M, vector<int>(N, -1));

    // Precompute hop penalty up to M+N
    const int maxDist = M + N;
    vector<double> hopPenalty(maxDist + 1);
    for (int d = 0; d <= maxDist; ++d) hopPenalty[d] = pow(2.0, -0.1 * d);

    // Read UAVs
    for (int i = 0; i < totalUAV; ++i) {
        int x, y, phi;
        double B;
        cin >> x >> y >> B >> phi;

        UAV u;
        u.x = x; u.y = y; u.phi = phi; u.B = B;

        for (int s = 0; s < 10; ++s) {
            if (s == 2 || s == 7) u.pattern[s] = B * 0.5;
            else if (s >= 3 && s <= 6) u.pattern[s] = B;
            else u.pattern[s] = 0.0;
        }

        u.prefix.assign(T + 1, 0.0);
        double cum = 0.0;
        for (int t = 0; t < T; ++t) {
            double bw = u.pattern[(u.phi + t) % 10];
            cum += bw;
            u.prefix[t + 1] = cum;
        }

        drones.push_back(std::move(u));
        uavIndex[x][y] = i;
    }

    vector<Flow> flows;
    flows.reserve(FN);

    // Read flows
    for (int i = 0; i < FN; ++i) {
        int f, sx, sy, tstart, m1, n1, m2, n2;
        double s;
        cin >> f >> sx >> sy >> tstart >> s >> m1 >> n1 >> m2 >> n2;

        Flow fl;
        fl.id = f;
        fl.sx = sx; fl.sy = sy;
        fl.start_time = tstart;
        fl.volume = s;
        fl.initial_volume = s;
        fl.last_landing_idx = -1;

        // Clamp region
        m1 = max(m1, 0); n1 = max(n1, 0);
        m2 = min(m2, M - 1); n2 = min(n2, N - 1);

        // Build candidates with cached distance & total capacity
        for (int X = m1; X <= m2; ++X) {
            for (int Y = n1; Y <= n2; ++Y) {
                int uidx = uavIndex[X][Y];
                if (uidx < 0) continue;
                int dist = abs(sx - X) + abs(sy - Y);
                double totalCap = drones[uidx].prefix[T] - drones[uidx].prefix[tstart];
                if (totalCap <= 1e-12) continue;
                fl.candidates.push_back({uidx, dist, hopPenalty[dist], totalCap});
            }
        }

        // Sort by totalCap * hopFactor (desc) — good primary choice
        sort(fl.candidates.begin(), fl.candidates.end(),
             [](const Flow::Candidate& a, const Flow::Candidate& b){
                 double ka = a.totalCap * a.hopFactor;
                 double kb = b.totalCap * b.hopFactor;
                 if (fabs(ka - kb) > 1e-9) return ka > kb;
                 if (a.dist != b.dist) return a.dist < b.dist;
                 return a.uav_idx < b.uav_idx;
             });

        flows.push_back(std::move(fl));
    }

    // Keep flows ordered by ID for stable output
    sort(flows.begin(), flows.end(), [](const Flow& a, const Flow& b){ return a.id < b.id; });

    // Tunables (aligned with scoring weights)
    const double W_CAP_NOW   = 0.55;  // instantaneous capacity
    const double W_LOOK      = 0.10;  // short lookahead
    const double W_DIST      = 0.32;  // hop factor
    const double W_STICK     = 0.03;  // prefer staying on same landing
    const int    LOOK_H      = 4;     // seconds of lookahead
    const int    FINISH_SWEEP = 20;   // aggressive final fill window (seconds)
    const double EPS         = 1e-12;

    // For lookahead, we’ll compute a tiny rolling sum on the fly.

    vector<int> active; active.reserve(FN);
    vector<double> capLeft(drones.size());

    auto uav_cap_at = [&](int uidx, int t)->double {
        return drones[uidx].pattern[(drones[uidx].phi + t) % 10];
    };

    auto uav_lookahead = [&](int uidx, int t)->double {
        double s = 0.0;
        for (int k = 0; k < LOOK_H && (t + k) < T; ++k) {
            s += uav_cap_at(uidx, t + k);
        }
        return s / (double)LOOK_H;
    };

    for (int t = 0; t < T; ++t) {
        // Build active set
        active.clear();
        active.reserve(FN);
        for (int i = 0; i < (int)flows.size(); ++i) {
            if (flows[i].volume > EPS && flows[i].start_time <= t)
                active.push_back(i);
        }
        if (active.empty()) continue;

        // Compute urgency for delay (delay term ≈ 10/(t+10), stronger than linear)
        // Prioritize: remaining_volume * urgency  (larger & earlier first)
        sort(active.begin(), active.end(), [&](int a, int b){
            const Flow &A = flows[a], &B = flows[b];
            double ua = exp(-0.10 * max(0, t - A.start_time)); // exponential decay
            double ub = exp(-0.10 * max(0, t - B.start_time));
            double pa = A.volume * ua;
            double pb = B.volume * ub;
            if (fabs(pa - pb) > 1e-9) return pa > pb;
            if (A.start_time != B.start_time) return A.start_time < B.start_time;
            return A.id < B.id;
        });

        // Init UAV capacities this second
        for (size_t u = 0; u < drones.size(); ++u) capLeft[u] = uav_cap_at((int)u, t);

        const bool in_finish_sweep = (t >= T - FINISH_SWEEP);

        // Allocate
        for (int idx : active) {
            Flow &F = flows[idx];
            if (F.volume <= EPS) continue;

            // Finish-sweep: push bytes anywhere within region to max throughput in last seconds
            if (in_finish_sweep) {
                // Prefer last landing if it has capacity
                if (F.last_landing_idx != -1 && capLeft[F.last_landing_idx] > EPS) {
                    double send = min(F.volume, capLeft[F.last_landing_idx]);
                    F.schedule.push_back({t, drones[F.last_landing_idx].x, drones[F.last_landing_idx].y, send});
                    F.volume -= send;
                    capLeft[F.last_landing_idx] -= send;
                    continue;
                }
                // Else, greedy over all candidates: pick max capacity now
                int bestU = -1; double bestCap = 0.0;
                for (const auto &c : F.candidates) {
                    if (capLeft[c.uav_idx] > bestCap) {
                        bestCap = capLeft[c.uav_idx];
                        bestU = c.uav_idx;
                    }
                }
                if (bestU != -1 && bestCap > EPS) {
                    double send = min(F.volume, bestCap);
                    F.schedule.push_back({t, drones[bestU].x, drones[bestU].y, send});
                    F.volume -= send;
                    capLeft[bestU] -= send;
                }
                continue; // one UAV per flow per second
            }

            // Normal mode: score candidates this second and choose argmax
            // If we already have a landing, try to stick to it with a small bias.
            int preferred = F.last_landing_idx;

            // Progress-aware distance emphasis (after most bytes done, push closer)
            double progress = (F.initial_volume <= 0 ? 1.0
                              : (1.0 - F.volume / F.initial_volume));
            double distWeight = (progress > 0.7 ? W_DIST * 2.0 : W_DIST);

            // First: quick try to stay on the same landing if it has capacity (reduces k)
            if (preferred != -1 && capLeft[preferred] > EPS) {
                double send = min(F.volume, capLeft[preferred]);
                F.schedule.push_back({t, drones[preferred].x, drones[preferred].y, send});
                F.volume -= send;
                capLeft[preferred] -= send;
                continue;
            }

            // Else: evaluate all candidates with remaining capacity
            int bestU = -1;
            double bestScore = -1e300;

            for (const auto &c : F.candidates) {
                int u = c.uav_idx;
                double capNow = capLeft[u];
                if (capNow <= EPS) continue;

                // Per-second utility approximating score gain:
                // cap importance + short lookahead + hop factor + tiny stick bonus
                double look = uav_lookahead(u, t);
                double stick = (u == preferred ? 1.0 : 0.0);

                double score =
                    W_CAP_NOW * capNow +
                    W_LOOK    * look   +
                    distWeight * c.hopFactor +
                    W_STICK   * stick;

                if (score > bestScore) {
                    bestScore = score;
                    bestU = u;
                }
            }

            if (bestU != -1) {
                double send = min(F.volume, capLeft[bestU]);
                F.schedule.push_back({t, drones[bestU].x, drones[bestU].y, send});
                F.volume -= send;
                capLeft[bestU] -= send;
                F.last_landing_idx = bestU;
            }
            // else: no candidate had capacity; do nothing this second.
        }
    }

    // Output
    cout.setf(std::ios::fixed);
    cout << setprecision(6);
    for (const Flow &F : flows) {
        cout << F.id << " " << F.schedule.size() << "\n";
        for (const auto &rec : F.schedule) {
            cout << rec.t << " " << rec.x << " " << rec.y << " " << rec.z << "\n";
        }
    }
    return 0;
}
