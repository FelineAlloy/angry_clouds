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
#include <unordered_set>
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
    int sx, sy;
    int start_time;
    double volume;
    double initial_volume;
    int last_landing_idx;

    // NEW:
    double Q_total;                // immutable total demand = initial_volume
    std::unordered_set<int> used_uavs; // which UAV indices we've ever used
    int k_distinct;                // = used_uavs.size(), cached so we don't keep recomputing

    struct Record { int t, x, y; double z; };
    vector<Record> schedule;

    struct Candidate {
        int uav_idx;
        int dist;
        double hopFactor; // 2^(-0.1 * dist)
        double totalCap;
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

    // ------------------------
    // Read UAVs (grid, pattern, prefix capacity)
    // ------------------------
    for (int i = 0; i < totalUAV; ++i) {
        int x, y, phi;
        double B;
        cin >> x >> y >> B >> phi;

        UAV u;
        u.x = x; u.y = y; u.phi = phi; u.B = B;

        // Bandwidth pattern repeating every 10 seconds
        // 0,1,8,9 -> 0
        // 2,7     -> B/2
        // 3..6    -> B
        for (int s = 0; s < 10; ++s) {
            if (s == 2 || s == 7) u.pattern[s] = B * 0.5;
            else if (s >= 3 && s <= 6) u.pattern[s] = B;
            else u.pattern[s] = 0.0;
        }

        // Prefix sum of future capacity, so we can query totalCap after tstart
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

    // ------------------------
    // Read flows (demands)
    // ------------------------
    vector<Flow> flows;
    flows.reserve(FN);

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

        // New scoring-related state:
        fl.Q_total = s;                // total demand baseline
        fl.used_uavs.clear();          // none used yet
        fl.k_distinct = 0;             // distinct landings so far

        // Clamp candidate region to grid
        m1 = max(m1, 0); n1 = max(n1, 0);
        m2 = min(m2, M - 1); n2 = min(n2, N - 1);

        // Precompute candidate UAVs for this flow
        for (int X = m1; X <= m2; ++X) {
            for (int Y = n1; Y <= n2; ++Y) {
                int uidx = uavIndex[X][Y];
                if (uidx < 0) continue;
                int dist = abs(sx - X) + abs(sy - Y);

                // How much this UAV can send from flow.start_time..T
                double totalCap = drones[uidx].prefix[T] - drones[uidx].prefix[tstart];
                if (totalCap <= 1e-12) continue;

                fl.candidates.push_back({
                    uidx,
                    dist,
                    hopPenalty[dist], // 2^(-0.1 * dist)
                    totalCap
                });
            }
        }

        // Order candidates by "long-term attractiveness"
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

    // Keep flows ordered in output-stable order
    sort(flows.begin(), flows.end(),
         [](const Flow& a, const Flow& b){ return a.id < b.id; });

    // ------------------------
    // Constants from scoring spec
    // ------------------------
    const double W_TRAFFIC    = 0.4;
    const double W_DELAY      = 0.2;
    const double W_DIST       = 0.3;
    const double W_STICK      = 0.1;
    const double ALPHA_DIST   = 0.1;    // distance decay α
    const double T_MAX_DELAY  = 10.0;   // Tmax in delay score
    const int    FINISH_SWEEP = 20;     // last seconds of desperation window
    const double EPS          = 1e-12;

    // Per-second working arrays
    vector<int>    active;
    vector<double> capLeft(drones.size());

    // capacity available from a UAV at a specific second
    auto uav_cap_at = [&](int uidx, int t)->double {
        return drones[uidx].pattern[(drones[uidx].phi + t) % 10];
    };

    // Commit actual transmission: update flow state, UAV remaining cap, stickiness info, and record output line
    auto commit_send = [&](Flow &F, int uidx, int tcur, double send_amount){
        F.schedule.push_back({tcur, drones[uidx].x, drones[uidx].y, send_amount});
        F.volume -= send_amount;
        if (F.volume < 0) F.volume = 0.0;
        capLeft[uidx] -= send_amount;
        if (capLeft[uidx] < 0) capLeft[uidx] = 0.0;

        F.last_landing_idx = uidx;
        if (F.used_uavs.find(uidx) == F.used_uavs.end()) {
            F.used_uavs.insert(uidx);
            F.k_distinct = (int)F.used_uavs.size();
        }
    };

    // Marginal score per Mbit if flow F sends right now via UAV uidx at time tcur.
    // Mirrors the spec:
    //   0.4 * (Δq/Qtot)
    // + 0.2 * (Tmax/(delay+Tmax)) * (Δq/Qtot)
    // + 0.3 * (Δq/Qtot) * 2^(-α*dist)
    // + 0.1 * (1/k_after)
    auto marginal_score_per_mbit = [&](const Flow &F, int uidx, int dist, int tcur)->double {
        double Qtot = (F.Q_total > 1e-12 ? F.Q_total : 1.0);

        double delay_since_start = std::max(0, tcur - F.start_time);

        // traffic delivered
        double traffic_term = W_TRAFFIC * (1.0 / Qtot);

        // earlier bytes get more credit
        double delay_weight = T_MAX_DELAY / (delay_since_start + T_MAX_DELAY);
        double delay_term   = W_DELAY * delay_weight * (1.0 / Qtot);

        // prefer nearer landings
        double hop_factor   = pow(2.0, -ALPHA_DIST * dist);
        double dist_term    = W_DIST * hop_factor * (1.0 / Qtot);

        // penalize creating new landing UAVs (k_distinct growth)
        bool already_used = (F.used_uavs.find(uidx) != F.used_uavs.end());
        int k_if_use = already_used ? F.k_distinct : (F.k_distinct + 1);
        double stickiness_value = (k_if_use > 0 ? (1.0 / (double)k_if_use) : 1.0);
        double stick_term = W_STICK * stickiness_value;

        return traffic_term + delay_term + dist_term + stick_term;
    };

    // "Neediness" = how urgently this flow needs help.
    // Intuition: if it's late in the timeline and this flow still has a lot left,
    // we should boost it to avoid it ending with an awful score.
    //
    // We'll return a multiplier in [~1 .. ~2+] so it's gentle, not explosive.
    auto flow_neediness = [&](const Flow &F, int tcur)->double {
        double time_left   = std::max(0, T - tcur);   // seconds remaining in horizon
        double progress    = (F.Q_total > 1e-12)
                             ? (1.0 - F.volume / F.Q_total) // fraction already delivered
                             : 1.0;
        // "behind" means: low progress & low time_left
        // scale ~ (1 + something). Example:
        double behind_factor = (1.0 - progress);            // high if we've delivered very little
        double urgency_time  = (time_left <= 0 ? 1.0
                                 : std::min(1.0, 20.0 / (double)time_left));
        // So if behind_factor ~1 and urgency_time ~1 -> boost ~2.
        double mult = 1.0 + 1.0 * behind_factor * urgency_time;
        return mult;
    };

    // Hysteresis threshold: how much better a NEW UAV must be than your current UAV
    // before we allow a switch (to protect stickiness).
    // We'll tune this small. You can tweak if needed.
    const double SWITCH_EPS = 1e-6;

    // We'll simulate for each second t
    for (int t = 0; t < T; ++t) {

        // Build list of flows that are active (started and not finished)
        active.clear();
        for (int i = 0; i < (int)flows.size(); ++i) {
            if (flows[i].volume > EPS && flows[i].start_time <= t) {
                active.push_back(i);
            }
        }
        if (active.empty()) continue;

        // Initialize UAV remaining capacity this second
        for (size_t u = 0; u < drones.size(); ++u) {
            capLeft[u] = uav_cap_at((int)u, t);
        }

        bool in_finish_sweep = (t >= T - FINISH_SWEEP);

        // -----------------------------
        // GLOBAL PER-SECOND MATCHING STEP
        //
        // Instead of:
        //   for each flow in priority order:
        //       pick best UAV
        //
        // We do:
        //   build ALL (flow,uav) candidate pairs
        //   compute global "benefit score"
        //   sort pairs by benefit
        //   greedily assign highest-benefit pairs first,
        //   respecting:
        //       - each flow can tx at most once this second
        //       - UAV capacity not exceeded
        //
        // This improves fairness and overall score.
        // -----------------------------

        struct PairChoice {
            int flow_idx;
            int uav_idx;
            int dist;
            double score_for_pair;   // score per Mbit * neediness
            double score_per_mbit;   // raw marginal per-mbit score (for hysteresis checks later)
        };

        vector<PairChoice> choices;
        choices.reserve(active.size() * 8); // heuristic reserve

        // Precompute neediness multiplier per flow now
        vector<double> flowNeed(active.size());
        for (size_t ai = 0; ai < active.size(); ++ai) {
            flowNeed[ai] = flow_neediness(flows[active[ai]], t);
        }

        // For each active flow, consider all UAV candidates
        for (size_t ai = 0; ai < active.size(); ++ai) {
            int fi = active[ai];
            Flow &F = flows[fi];
            if (F.volume <= EPS) continue;

            for (const auto &cand : F.candidates) {
                int u = cand.uav_idx;
                if (capLeft[u] <= EPS) continue; // UAV has nothing left this second

                double base_per_mbit = marginal_score_per_mbit(F, u, cand.dist, t);

                // Neediness multiplier:
                double mult = flowNeed[ai];

                double boosted = base_per_mbit * mult;

                // We'll push this pair for later ranking
                choices.push_back({
                    fi,
                    u,
                    cand.dist,
                    boosted,
                    base_per_mbit
                });
            }
        }

        // Sort all possible (flow,uav) pairs by DESC score_for_pair
        sort(choices.begin(), choices.end(),
             [&](const PairChoice &A, const PairChoice &B){
                 if (fabs(A.score_for_pair - B.score_for_pair) > 1e-12)
                     return A.score_for_pair > B.score_for_pair;
                 // tie-break: earlier start time preferred, then lower id
                 const Flow &FA = flows[A.flow_idx];
                 const Flow &FB = flows[B.flow_idx];
                 if (FA.start_time != FB.start_time) return FA.start_time < FB.start_time;
                 return FA.id < FB.id;
             });

        // Track which flows have already been served this second
        vector<char> flowSentThisSecond(flows.size(), 0);

        // We will also apply hysteresis (don't switch UAVs for tiny gain)
        // when we actually perform assignment below.

        // Greedily assign pairs in sorted order
        for (const auto &pc : choices) {
            int fi  = pc.flow_idx;
            int ui  = pc.uav_idx;
            Flow &F = flows[fi];

            // Check flow still valid:
            if (F.volume <= EPS) continue;
            if (flowSentThisSecond[fi]) continue;          // flow already transmitted this second
            if (capLeft[ui] <= EPS) continue;               // UAV ran out

            // Hysteresis / anti-flap rule:
            // If F already has a last_landing_idx (some UAV it used earlier)
            // AND we're about to switch to a brand-new UAV,
            // demand that this new UAV is clearly better.
            //
            // We'll compare pc.score_per_mbit vs score_per_mbit for staying on last_landing_idx.
            if (F.last_landing_idx != -1 && F.last_landing_idx != ui) {
                // score of sticking with old UAV (if old UAV still has cap)
                int oldU = F.last_landing_idx;
                double oldCap = capLeft[oldU];
                if (oldCap > EPS) {
                    // we need the dist for oldU; get it from F.candidates
                    int oldDist = 0;
                    bool foundOld = false;
                    for (const auto &c2 : F.candidates) {
                        if (c2.uav_idx == oldU) {
                            oldDist = c2.dist;
                            foundOld = true;
                            break;
                        }
                    }
                    if (foundOld) {
                        double stay_score = marginal_score_per_mbit(F, oldU, oldDist, t);
                        // If switching doesn't beat staying by some margin, skip this pair
                        // (protects stickiness and prevents noisy micro-handoffs).
                        if (pc.score_per_mbit <= stay_score + SWITCH_EPS) {
                            continue;
                        }
                    }
                }
            }

            // How much can we send?
            double send_amt = std::min(F.volume, capLeft[ui]);
            if (send_amt <= EPS) continue;

            // Commit the transmission
            commit_send(F, ui, t, send_amt);

            // Mark that this flow is done for this second (one UAV per flow per second)
            flowSentThisSecond[fi] = 1;

            // If we're in the very last seconds (finish sweep), we could consider
            // letting the same flow grab multiple UAVs in the same second
            // *if* they were already used UAVs, but for now we keep the single-assignment
            // rule because your output format assumes max 1 landing per second entry.
        }

        // loop next second...
    }

    // ------------------------
    // Output final schedule
    // ------------------------
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