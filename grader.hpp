#ifndef SCORER_HPP
#define SCORER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility> // For std::pair

[cite_start]// Structure to hold initial information about each flow from the input file[cite: 55, 66].
struct FlowInfo {
    int id;
    int accessX, accessY;
    int startTime;
    double qTotal;
};

[cite_start]// Structure to hold a single scheduling record from the output file[cite: 70].
struct ScheduleRecord {
    int t; // Absolute time
    int landingX, landingY;
    double trafficAmount; // z, traffic amount in Mbits for this time slot
};

/**
 * @brief Calculates the final score based on an input and output file.
 *
 * This function parses the flow definitions from the input file and the
 * scheduling results from the output file to calculate a score according to
 * [cite_start]the provided scoring rules[cite: 81, 100].
 *
 * @param inputFile Path to the input file containing network and flow definitions.
 * @param outputFile Path to the output file containing the scheduling results.
 * @return The final calculated score as a double.
 */
double calculateScore(const std::string& inputFile, const std::string& outputFile) {
    std::ifstream inFile(inputFile);
    if (!inFile.is_open()) {
        throw std::runtime_error("Error: Could not open input file: " + inputFile);
    }

    std::ifstream outFile(outputFile);
    if (!outFile.is_open()) {
        throw std::runtime_error("Error: Could not open output file: " + outputFile);
    }

    // --- 1. Parse Input File ---
    std::map<int, FlowInfo> flowData;
    int M, N, FN, T;
    inFile >> M >> N >> FN >> T; [cite_start]// [cite: 62]

    [cite_start]// Skip the M*N lines of UAV bandwidth information[cite: 64].
    for (int i = 0; i < M * N; ++i) {
        std::string line;
        int x;
        if (inFile >> x) {
             getline(inFile, line);
        }
    }

    [cite_start]// Read flow information[cite: 66].
    for (int i = 0; i < FN; ++i) {
        FlowInfo info;
        int m1, n1, m2, n2; // Landing range, not needed for scoring
        inFile >> info.id >> info.accessX >> info.accessY >> info.startTime >> info.qTotal >> m1 >> n1 >> m2 >> n2;
        flowData[info.id] = info;
    }
    inFile.close();

    // --- 2. Parse Output File ---
    std::map<int, std::vector<ScheduleRecord>> scheduleData;
    int flowId, p;
    [cite_start]// Read flow ID and the number of scheduling records for it[cite: 69].
    while (outFile >> flowId >> p) {
        std::vector<ScheduleRecord> records;
        [cite_start]// Read each scheduling record[cite: 70].
        for (int i = 0; i < p; ++i) {
            ScheduleRecord record;
            outFile >> record.t >> record.landingX >> record.landingY >> record.trafficAmount;
            records.push_back(record);
        }
        scheduleData[flowId] = records;
    }
    outFile.close();

    // --- 3. Calculate Scores ---
    double totalWeightedScore = 0.0;
    double totalTrafficOfAllFlows = 0.0;

    // Iterate over each flow that has a schedule in the output file
    for (const auto& pair : scheduleData) {
        int currentFlowId = pair.first;
        const auto& records = pair.second;
        
        if (flowData.find(currentFlowId) == flowData.end()) {
            continue; // Skip if flow from output is not in input definitions
        }
        const FlowInfo& info = flowData.at(currentFlowId);

        double totalU2GTraffic = 0.0;
        double delayScoreComponent = 0.0;
        double distanceScoreComponent = 0.0;
        std::set<std::pair<int, int>> uniqueLandingPoints;

        // The constants for scoring formulas
        const double T_MAX = 10.0; [cite_start]// [cite: 88]
        const double ALPHA = 0.1;  [cite_start]// [cite: 92]

        for (const auto& rec : records) {
            double qi = rec.trafficAmount;
            totalU2GTraffic += qi;
            uniqueLandingPoints.insert({rec.landingX, rec.landingY});

            [cite_start]// Delay Score calculation[cite: 89]. Uses relative time (t - t_start).
            double relative_t = rec.t - info.startTime;
            delayScoreComponent += (T_MAX / (relative_t + T_MAX)) * (qi / info.qTotal);

            [cite_start]// Distance Score calculation[cite: 91].
            [cite_start]// Manhattan distance (number of hops)[cite: 90].
            int di = std::abs(rec.landingX - info.accessX) + std::abs(rec.landingY - info.accessY);
            distanceScoreComponent += (qi / info.qTotal) * std::pow(2, -ALPHA * di);
        }

        // Calculate the four component scores for the current flow
        double totalU2GTrafficScore = (info.qTotal > 0) ? (totalU2GTraffic / info.qTotal) : 0.0; [cite_start]// [cite: 84]
        double landingUAVPointScore = uniqueLandingPoints.empty() ? 0.0 : (1.0 / uniqueLandingPoints.size()); [cite_start]// [cite: 95]
        
        [cite_start]// Calculate the final score for this flow using the specified weights[cite: 97].
        double flowScore = 100.0 * (
            [cite_start]0.4 * totalU2GTrafficScore +   // Weight: 0.4 [cite: 87]
            [cite_start]0.2 * delayScoreComponent +    // Weight: 0.2 [cite: 88]
            [cite_start]0.3 * distanceScoreComponent + // Weight: 0.3 [cite: 92]
            [cite_start]0.1 * landingUAVPointScore     // Weight: 0.1 [cite: 94]
        );

        totalWeightedScore += flowScore * info.qTotal;
        totalTrafficOfAllFlows += info.qTotal;
    }

    if (totalTrafficOfAllFlows == 0) {
        return 0.0;
    }

    [cite_start]// Final score is the weighted average of all flow scores[cite: 100].
    return totalWeightedScore / totalTrafficOfAllFlows;
}

#endif // SCORER_HPP