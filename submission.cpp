#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <climits>
#include <limits.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

/*** -------------------- Problem Types -------------------- ***/
struct TxRec {
    int t, x, y;
    double z; // Mbps sent in second t
};

struct Drone {
    int x, y;
    double B;
    int phi;

    // bandwidth already allocated at time t on this UAV
    unordered_map<int, double> used; // t -> used Mbps

    inline int slot(int t) const { return (t + phi) % 10; }

    inline double base_bw_at_slot(int s) const {
        // Period: 0..9
        // 0,1,8,9 => 0; 2,7 => B/2; 3..6 => B
        if (s == 2 || s == 7) return B * 0.5;
        if (s >= 3 && s <= 6) return B;
        return 0.0;
    }

    inline double base_bw(int t) const {
        return base_bw_at_slot(slot(t));
    }

    inline double avail(int t) const {
        double cap = base_bw(t);
        auto it = used.find(t);
        if (it != used.end()) cap -= it->second;
        if (cap < 0) cap = 0;
        return cap;
    }

    // average base bandwidth over next horizon seconds (phase-aware, no contention)
    inline double avg_bw_lookahead(int t, int horizon = 5) const {
        double s = 0.0;
        for (int k = 0; k < horizon; ++k) s += base_bw(t + k);
        return s / horizon;
    }
};

struct Flow {
    int id;
    int sx, sy;     // access UAV (source)
    int t_start;
    double s_left;  // remaining Mbits
    int m1, n1, m2, n2; // landing rectangle (inclusive)
    int last_x = INT_MIN, last_y = INT_MIN; // last chosen landing UAV (for stickiness)
    vector<TxRec> out;

    // cache candidate UAV indexes for faster per-step selection
    vector<int> candidates;
};

/*** -------------------- Helpers -------------------- ***/
static inline int manhattan(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

//int argc, char* argv[]
int main(int argc, char* argv[]) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int M, N, FN, T;
    if (!(cin >> M >> N >> FN >> T)) return 0;

    // Read constants from command line (for tuning)
    // double W_BW = 0.4, W_LOOK = 0.1, W_DIST = 0.4, W_STICK = 0.1, ALPHA_URG = 0.0;
    // if (argc >= 6) {
    //     W_BW      = atof(argv[1]);
    //     W_LOOK    = atof(argv[2]);
    //     W_DIST    = atof(argv[3]);
    //     W_STICK   = atof(argv[4]);
    //     ALPHA_URG = atof(argv[5]);
    // }

    const int D = M * N;
    vector<Drone> drones;
    drones.reserve(D);

    // index map (x,y) -> index
    vector<vector<int>> idxOf(M, vector<int>(N, -1));

    for (int i = 0; i < D; ++i) {
        int x, y, phi;
        double B;
        cin >> x >> y >> B >> phi;
        drones.push_back({x, y, B, phi, {}});
        idxOf[x][y] = i;
    }

    vector<Flow> flows;
    flows.reserve(FN);

    for (int i = 0; i < FN; ++i) {
        int f, x, y, tstart, m1, n1, m2, n2;
        double s;
        cin >> f >> x >> y >> tstart >> s >> m1 >> n1 >> m2 >> n2;
        Flow fl{f, x, y, tstart, s, m1, n1, m2, n2};
        // Precompute candidate UAV indices within rectangle
        for (int X = m1; X <= m2; ++X)
            for (int Y = n1; Y <= n2; ++Y)
                if (X >= 0 && X < M && Y >= 0 && Y < N)
                    fl.candidates.push_back(idxOf[X][Y]);
        flows.push_back(std::move(fl));
    }

    // bw = 0.4, dist = 0.4, stick = 0.2
    // ---- Tunable weights (reflecting score formula) ----
    const double W_BW      = 0.2;  // instantaneous available bandwidth
    const double W_LOOK    = 0.25;  // 10s lookahead average
    const double W_DIST    = 0.6;  // prefer short distance (higher score)
    const double W_STICK   = 0.125;  // avoid switching landing UAVs
    const double ALPHA_URG = 0.05;  // urgency decay per second since t_start
    const double EPS       = 1e-12;

    // precompute max B among drones to normalize bandwidth terms
    double Bmax = 0.0;
    for (auto &d : drones) Bmax = max(Bmax, d.B > 0 ? d.B : 0.0);
    if (Bmax < EPS) Bmax = 1.0;

    // To improve delay + fairness: process active flows in SRPT-ish order each second
    vector<int> activeIdx; activeIdx.reserve(FN);

    for (int t = 0; t < T; ++t) {
        activeIdx.clear();
        for (int i = 0; i < FN; ++i) {
            if (flows[i].s_left > EPS && flows[i].t_start <= t) activeIdx.push_back(i);
        }

        // Sort by "tightness": smaller remaining first (SRPT flavor)
        sort(activeIdx.begin(), activeIdx.end(), [&](int a, int b){
            if (flows[a].s_left == flows[b].s_left) return flows[a].id < flows[b].id;
            return flows[a].s_left < flows[b].s_left;
        });

        // For each active flow, pick best landing UAV using multi-objective utility
        for (int fi : activeIdx) {
            Flow &F = flows[fi];
            if (F.s_left <= EPS) continue;

            double bestScore = -1e100;
            int bestDroneIdx = -1;

            // urgency factor to bias earlier transmission (improves delay score)
            double urg = 1.0 / (1.0 + ALPHA_URG * max(0, t - F.t_start));

            for (int dIdx : F.candidates) {
                const Drone &D = drones[dIdx];
                // skip if base is zero at this slot to prune search
                if (D.base_bw(t) <= EPS) continue;

                double avail = D.avail(t);           // normalized by Bmax next
                if (avail <= EPS) continue;

                double look = D.avg_bw_lookahead(t); // 0..B
                int dist = manhattan(F.sx, F.sy, D.x, D.y);
                double distTerm = 1.0 / (1.0 + (double)dist); // higher is better (shorter)

                double stick = (F.last_x == D.x && F.last_y == D.y) ? 1.0 : 0.0;

                double score =
                    W_BW   * (avail / Bmax) +
                    W_LOOK * (look  / Bmax) +
                    W_DIST * distTerm +
                    W_STICK* stick;

                score *= urg;

                if (score > bestScore) {
                    bestScore = score;
                    bestDroneIdx = dIdx;
                }
            }

            if (bestDroneIdx == -1) {
                // No capacity anywhere in window this second; skip.
                continue;
            }

            Drone &chosen = drones[bestDroneIdx];
            double canSend = min(chosen.avail(t), F.s_left);
            if (canSend > EPS) {
                chosen.used[t] += canSend;
                F.s_left -= canSend;

                // update stickiness anchor
                F.last_x = chosen.x;
                F.last_y = chosen.y;

                F.out.push_back({t, chosen.x, chosen.y, canSend});
            }
        }
    }

    // ---- Output per spec ----
    cout.setf(std::ios::fixed); cout << setprecision(6);
    for (const auto &F : flows) {
        cout << F.id << " " << F.out.size() << "\n";
        for (const auto &rec : F.out) {
            cout << rec.t << " " << rec.x << " " << rec.y << " " << rec.z << "\n";
        }
    }

    return 0;
}
