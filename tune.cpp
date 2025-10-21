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

int main() {
    // search ranges
    vector<double> w_bw   = {0.2, 0.4, 0.5, 0.6, 0.7, 0.8};
    vector<double> w_look = {0.0, 0.15, 0.175, 0.2, 0.25};
    vector<double> w_dist = {0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6};
    vector<double> w_stick= {0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2};
    vector<double> alpha  = {0.0, 0.05, 0.1, 0.125, 0.15};

    string input = "in1.in";

    double bestScore = -1e9;
    string bestParams;

    // compile once (fast)
    system("g++ -O2 -std=gnu++17 submission.cpp -o submission >/dev/null 2>&1");

    for (double bw : w_bw)
    for (double lk : w_look)
    for (double ds : w_dist)
    for (double st : w_stick)
    for (double al : alpha) {

        // run program
        string cmd = "./submission " + to_string(bw) + " " + to_string(lk) + " " +
                     to_string(ds) + " " + to_string(st) + " " + to_string(al) +
                     " < " + input + " > tmp.out";
        system(cmd.c_str());

        // run local scoring script (you’ll implement)
        double score = 0.0;
        // Example: call a python script "score.py tmp.out"
        // FILE* f = popen("python3 score.py tmp.out", "r");
        // fscanf(f, "%lf", &score); pclose(f);

        // For now, fake a random score placeholder
        score = (rand() % 1000) / 10.0;

        cout << "Params: " << bw << "," << lk << "," << ds << "," << st << "," << al
             << " -> Score: " << score << "\n";

        if (score > bestScore) {
            bestScore = score;
            bestParams = cmd;
        }
    }

    cout << "\nBest score: " << bestScore << "\n";
    cout << "Best params: " << bestParams << "\n";
    return 0;
}
