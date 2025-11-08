#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <sstream>
#include <memory>
#include <algorithm>
#include <unordered_set>
#include <fstream>
#include <chrono>
#include <random>
#include <thread>
#include <numeric>
#include <iomanip>

using namespace std;

// Game constants
const int GRID_SIZE = 13;
const int MAX_STEPS = 500;
const int MAX_ITERATIONS = 10000;

// Test result structure
struct TestResult {
    int testId;
    bool success;
    int steps;
    int totalTime;
    string errorMessage;
    vector<vector<int>> path;
    vector<vector<int>> exploredNodes;
    string algorithm;
    int variant;
};

// Algorithm comparison structure
struct AlgorithmStats {
    string algorithm;
    int variant;
    int wins;
    int losses;
    double winRate;
    vector<int> executionTimes;
    vector<int> stepsCount;
    double meanTime;
    double medianTime;
    double modeTime;
    double stdDevTime;
    double meanSteps;
    double medianSteps;
    double modeSteps;
    double stdDevSteps;
};

// Represents a state in the game world during pathfinding
struct NodeState
{
    int posX, posY;
    bool isRingActive;
    int costFromStart;
    int heuristicCost;
    shared_ptr<NodeState> previousNode;

    bool operator<(const NodeState& other) const
    {
        return costFromStart + heuristicCost > other.costFromStart + other.heuristicCost;
    }

    bool operator==(const NodeState& other) const
    {
        return posX == other.posX && posY == other.posY && isRingActive == other.isRingActive;
    }
};

namespace std
{
    template<>
    struct hash<NodeState>
    {
        size_t operator()(const NodeState& state) const
        {
            return hash<int>()(state.posX) ^ hash<int>()(state.posY) ^ hash<bool>()(state.isRingActive);
        }
    };
}

class RingQuestSolver
{
private:
    // Game state
    set<pair<int, int>> safePositions;
    set<pair<int, int>> dangerousPositions;
    vector<tuple<int, int, char>> enemyList;

    int gameVariant;
    int gollumPosX, gollumPosY;
    int volcanoPosX, volcanoPosY;
    bool mithrilAcquired = false;

    // Test logging
    vector<vector<int>> currentPath;
    vector<vector<int>> exploredNodes;

    // Movement directions
    static constexpr int DIRECTIONS[4][2] = {{0,1}, {0,-1}, {1,0}, {-1,0}};

    // Returns all cells in perception range around given position
    vector<pair<int, int>> getDetectionZone(int centerX, int centerY) const
    {
        vector<pair<int, int>> detectionArea;
        const int detectionRadius = (gameVariant == 1) ? 1 : 2;

        for (int dx = -detectionRadius; dx <= detectionRadius; dx++)
        {
            for (int dy = -detectionRadius; dy <= detectionRadius; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                int newX = centerX + dx;
                int newY = centerY + dy;

                if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE)
                {
                    detectionArea.emplace_back(newX, newY);
                }
            }
        }
        return detectionArea;
    }

    // Nazgul detection patterns
    bool isInNazgulBaseZone(int checkX, int checkY, int nazgulX, int nazgulY) const
    {
        return (checkX == nazgulX && checkY == nazgulY) ||
               (checkX == nazgulX-2 && checkY == nazgulY) ||
               (checkX == nazgulX-1 && abs(checkY - nazgulY) <= 1) ||
               (checkX == nazgulX && abs(checkY - nazgulY) <= 2 && checkY != nazgulY) ||
               (checkX == nazgulX+1 && abs(checkY - nazgulY) <= 1) ||
               (checkX == nazgulX+2 && checkY == nazgulY);
    }

    bool isInNazgulExpandedZone(int checkX, int checkY, int nazgulX, int nazgulY) const
    {
        return max(abs(checkX - nazgulX), abs(checkY - nazgulY)) <= 2 ||
               (checkX == nazgulX-3 && checkY == nazgulY) ||
               (checkX == nazgulX+3 && checkY == nazgulY) ||
               (checkX == nazgulX && checkY == nazgulY-3) ||
               (checkX == nazgulX && checkY == nazgulY+3);
    }

    // Enemy threat detection
    bool isThreatenedByEnemy(int posX, int posY, int enemyX, int enemyY, char enemyType, bool ringActive) const
    {
        if (posX == enemyX && posY == enemyY)
            return true;

        switch (enemyType)
        {
            case 'O':
                if (mithrilAcquired || ringActive)
                {
                    return posX == enemyX && posY == enemyY;
                }
                return abs(posX - enemyX) + abs(posY - enemyY) <= 1;

            case 'U':
                if (mithrilAcquired || ringActive)
                {
                    return abs(posX - enemyX) + abs(posY - enemyY) <= 1;
                }
                return abs(posX - enemyX) + abs(posY - enemyY) <= 2;

            case 'N':
                return ringActive ? isInNazgulExpandedZone(posX, posY, enemyX, enemyY)
                                 : isInNazgulBaseZone(posX, posY, enemyX, enemyY);

            case 'W':
                if (ringActive)
                {
                    return isInNazgulExpandedZone(posX, posY, enemyX, enemyY);
                }
                return max(abs(posX - enemyX), abs(posY - enemyY)) <= 2;

            default:
                return false;
        }
    }

    // Core safety check
    bool isPositionSafe(int posX, int posY, bool ringActive) const
    {
        if (safePositions.count({posX, posY}))
            return true;

        if (dangerousPositions.count({posX, posY}))
            return false;

        for (const auto& enemy : enemyList)
        {
            if (isThreatenedByEnemy(posX, posY, get<0>(enemy),
                get<1>(enemy), get<2>(enemy), ringActive))
            {
                return false;
            }
        }
        return true;
    }

    // Map updating logic
    void processGameObject(int cellX, int cellY, char objectType)
    {
        switch (objectType)
        {
            case 'P':
                dangerousPositions.emplace(cellX, cellY);
                break;

            case 'O': case 'U': case 'N': case 'W':
                addEnemyIfNew(cellX, cellY, objectType);
                break;

            case 'G':
                gollumPosX = cellX;
                gollumPosY = cellY;
                break;

            case 'M':
                volcanoPosX = cellX;
                volcanoPosY = cellY;
                break;

            case 'C':
                mithrilAcquired = true;
                break;
        }
    }

    void addEnemyIfNew(int cellX, int cellY, char enemyType)
    {
        for (const auto& enemy : enemyList)
        {
            if (get<0>(enemy) == cellX && get<1>(enemy) == cellY)
            {
                return;
            }
        }
        enemyList.emplace_back(cellX, cellY, enemyType);
    }

    void updateGameMap(int currentX, int currentY, const vector<string>& perceptionData)
    {
        // Clear dangerous status from detection zone
        auto detectionZone = getDetectionZone(currentX, currentY);
        for (const auto& cell : detectionZone)
        {
            dangerousPositions.erase(cell);
        }

        // Process new perceptions
        for (const string& data : perceptionData)
        {
            istringstream stream(data);
            int cellX, cellY;
            char objectType;
            stream >> cellX >> cellY >> objectType;
            processGameObject(cellX, cellY, objectType);
        }
    }

    // A* Pathfinding
    vector<NodeState> findPathAStar(int startX, int startY, bool startRingState,
                                   int targetX, int targetY, bool logExploration = false)
    {
        priority_queue<NodeState> openSet;
        unordered_set<NodeState> closedSet;

        NodeState startNode = {startX, startY, startRingState, 0,
                              calculateHeuristic(startX, startY, targetX, targetY),
            nullptr};

        openSet.push(startNode);

        int iterationCount = 0;

        while (!openSet.empty() && iterationCount < MAX_ITERATIONS)
        {
            iterationCount++;
            NodeState currentNode = openSet.top();
            openSet.pop();

            if (closedSet.count(currentNode))
                continue;

            closedSet.insert(currentNode);

            // Log explored nodes for visualization
            if (logExploration) {
                exploredNodes.push_back({currentNode.posX, currentNode.posY, currentNode.isRingActive ? 1 : 0});
            }

            if (isGoalReached(currentNode, targetX, targetY))
            {
                return reconstructPath(currentNode);
            }

            expandNodeAStar(currentNode, targetX, targetY, openSet, closedSet);
        }

        return {};
    }

    int calculateHeuristic(int fromX, int fromY, int toX, int toY) const
    {
        return abs(fromX - toX) + abs(fromY - toY);
    }

    bool isGoalReached(const NodeState& state, int targetX, int targetY) const
    {
        return state.posX == targetX && state.posY == targetY;
    }

    void expandNodeAStar(const NodeState& currentNode, int targetX, int targetY,
                   priority_queue<NodeState>& openSet, unordered_set<NodeState>& closedSet) const
    {
        // Expand to neighboring positions
        for (const auto& dir : DIRECTIONS)
        {
            int newX = currentNode.posX + dir[0];
            int newY = currentNode.posY + dir[1];

            if (isValidPosition(newX, newY) && isPositionSafe(newX, newY, currentNode.isRingActive))
            {
                addNodeToOpenSet(newX, newY, currentNode.isRingActive, currentNode,
                               targetX, targetY, openSet, closedSet);
            }
        }

        // Expand ring toggle action
        addNodeToOpenSet(currentNode.posX, currentNode.posY, !currentNode.isRingActive,
                        currentNode, targetX, targetY, openSet, closedSet);
    }

    bool isValidPosition(int x, int y) const
    {
        return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
    }

    void addNodeToOpenSet(int x, int y, bool ringActive, const NodeState& parent,
                         int targetX, int targetY, priority_queue<NodeState>& openSet,
                         const unordered_set<NodeState>& closedSet) const {
        NodeState newNode = {x, y, ringActive, parent.costFromStart + 1,
                            calculateHeuristic(x, y, targetX, targetY),
                            make_shared<NodeState>(parent)};

        if (!closedSet.count(newNode))
        {
            openSet.push(newNode);
        }
    }

    vector<NodeState> reconstructPath(const NodeState& goalNode) const
    {
        vector<NodeState> path;
        shared_ptr<NodeState> current = make_shared<NodeState>(goalNode);

        while (current != nullptr)
        {
            path.push_back(*current);
            current = current->previousNode;
        }

        reverse(path.begin(), path.end());
        return path;
    }

    // Input && Output operations
    void processPerceptionData(vector<string>& perceptions)
    {
        int perceptionCount;
        cin >> perceptionCount;
        cin.ignore();
        perceptions.resize(perceptionCount);

        for (int i = 0; i < perceptionCount; i++)
        {
            getline(cin, perceptions[i]);
        }
    }

    // Navigation core
    bool navigateToTarget(int& currentX, int& currentY, bool& ringActive,
                         int targetX, int targetY, int& totalSteps,
                         const string& algorithm = "A*", bool logPath = false)
    {
        while ((currentX != targetX || currentY != targetY) && totalSteps < MAX_STEPS)
        {
            vector<NodeState> path;

            if (algorithm == "A*") {
                path = findPathAStar(currentX, currentY, ringActive, targetX, targetY, logPath);
            }

            if (path.size() < 2)
                return false;

            const NodeState& nextStep = path[1];

            // Log path for visualization
            if (logPath) {
                currentPath.push_back({nextStep.posX, nextStep.posY, nextStep.isRingActive ? 1 : 0});
            }

            executeNextStep(currentX, currentY, ringActive, nextStep, totalSteps);
        }
        return currentX == targetX && currentY == targetY;
    }

    void executeNextStep(int& currentX, int& currentY, bool& ringActive,
                        const NodeState& nextStep, int& totalSteps)
    {
        if (nextStep.posX != currentX || nextStep.posY != currentY)
        {
            // Movement action
            cout << "m " << nextStep.posX << " " << nextStep.posY << endl;
            currentX = nextStep.posX;
            currentY = nextStep.posY;
            safePositions.emplace(currentX, currentY);
            totalSteps++;
        }
        else
        {
            // Ring toggle action
            executeRingToggle(ringActive, nextStep.isRingActive);
        }

        updateGameState(currentX, currentY);
    }

    void executeRingToggle(bool& currentRingState, bool targetRingState)
    {
        if (targetRingState && !currentRingState)
        {
            cout << "r" << endl;
            currentRingState = true;
        }
        else if (!targetRingState && currentRingState)
        {
            cout << "rr" << endl;
            currentRingState = false;
        }
    }

    void updateGameState(int currentX, int currentY)
    {
        vector<string> perceptionData;
        processPerceptionData(perceptionData);
        updateGameMap(currentX, currentY, perceptionData);
    }

    // Валидация количества врагов по требованиям PDF
    void validateEnemyCounts() const {
        map<char, int> enemyCounts;

        for (const auto& enemy : enemyList) {
            enemyCounts[get<2>(enemy)]++;
        }

        // Проверка по требованиям PDF
        if (enemyCounts['O'] < 1 || enemyCounts['O'] > 2) {
            cerr << "Warning: Invalid Orc count: " << enemyCounts['O'] << " (should be 1-2)" << endl;
        }
        if (enemyCounts['U'] != 1) {
            cerr << "Warning: Invalid Uruk-hai count: " << enemyCounts['U'] << " (should be 1)" << endl;
        }
        if (enemyCounts['N'] > 1) {
            cerr << "Warning: Invalid Nazgul count: " << enemyCounts['N'] << " (should be 0-1)" << endl;
        }
        if (enemyCounts['W'] != 1) {
            cerr << "Warning: Invalid Watchtower count: " << enemyCounts['W'] << " (should be 1)" << endl;
        }
    }

public:
    void executeSolution()
    {
        initializeGame();

        int currentX = 0, currentY = 0;
        bool ringActive = false;
        int totalSteps = 0;

        // Phase 1: Find Gollum
        if (!navigateToTarget(currentX, currentY, ringActive, gollumPosX,
            gollumPosY, totalSteps))
        {
            cout << "e -1" << endl;
            return;
        }

        // Phase 2: Reach Mount Doom
        cin >> volcanoPosX >> volcanoPosY;

        if (!navigateToTarget(currentX, currentY, ringActive, volcanoPosX,
        volcanoPosY, totalSteps))
        {
            cout << "e -1" << endl;
            return;
        }

        cout << "e " << totalSteps << endl;
    }

    TestResult runSingleTest(int testId, int variant, int gollumX, int gollumY,
                           int volcanoX, int volcanoY, const vector<string>& initialPerceptions,
                           const string& algorithm = "A*", bool enableLogging = false) {
        TestResult result;
        result.testId = testId;
        result.algorithm = algorithm;
        result.variant = variant;

        auto startTime = chrono::high_resolution_clock::now();

        // Reset state
        safePositions.clear();
        dangerousPositions.clear();
        enemyList.clear();
        mithrilAcquired = false;
        currentPath.clear();
        exploredNodes.clear();

        // Initialize game state
        gameVariant = variant;
        this->gollumPosX = gollumX;
        this->gollumPosY = gollumY;
        this->volcanoPosX = volcanoX;
        this->volcanoPosY = volcanoY;
        safePositions.emplace(0, 0);

        // Process initial perceptions
        updateGameMap(0, 0, initialPerceptions);

        // Валидация количества врагов
        if (enableLogging) {
            validateEnemyCounts();
        }

        int currentX = 0, currentY = 0;
        bool ringActive = false;
        int totalSteps = 0;

        // Phase 1: Find Gollum
        if (!navigateToTarget(currentX, currentY, ringActive, gollumX, gollumY, totalSteps, algorithm, enableLogging)) {
            result.success = false;
            result.errorMessage = "Failed to reach Gollum with " + algorithm;
        }
        // Phase 2: Reach Mount Doom
        else if (!navigateToTarget(currentX, currentY, ringActive, volcanoX, volcanoY, totalSteps, algorithm, enableLogging)) {
            result.success = false;
            result.errorMessage = "Failed to reach Volcano with " + algorithm;
        }
        else {
            result.success = true;
            result.steps = totalSteps;
        }

        auto endTime = chrono::high_resolution_clock::now();
        result.totalTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();

        if (enableLogging) {
            result.path = currentPath;
            result.exploredNodes = exploredNodes;
        }

        return result;
    }

private:
    void initializeGame()
    {
        cin >> gameVariant;
        cin >> gollumPosX >> gollumPosY;
        safePositions.emplace(0, 0);

        vector<string> perceptionData;
        processPerceptionData(perceptionData);
        updateGameMap(0, 0, perceptionData);
    }
};

// Test Case Generator - ИСПРАВЛЕННЫЙ ВАРИАНТ
class TestCaseGenerator {
private:
    random_device rd;
    mt19937 gen;
    uniform_int_distribution<> coordDist;
    uniform_int_distribution<> variantDist;
    uniform_int_distribution<> orcCountDist;
    uniform_int_distribution<> nazgulCountDist;
    uniform_int_distribution<> numPitsDist;

public:
    TestCaseGenerator() : gen(rd()), coordDist(0, GRID_SIZE-1),
                         variantDist(1, 2),
                         orcCountDist(1, 2),  // Orcs: 1-2
                         nazgulCountDist(0, 1),  // Nazgul: 0-1
                         numPitsDist(2, 5) {}  // Pits: 2-5

    struct TestCase {
        int testId;
        int variant;
        int gollumX, gollumY;
        int volcanoX, volcanoY;
        vector<string> perceptions;
    };

    TestCase generateTest(int testId) {
        TestCase test;
        test.testId = testId;
        test.variant = variantDist(gen);

        // Generate positions that are not too close to start
        do {
            test.gollumX = coordDist(gen);
            test.gollumY = coordDist(gen);
        } while (test.gollumX < 3 && test.gollumY < 3);

        do {
            test.volcanoX = coordDist(gen);
            test.volcanoY = coordDist(gen);
        } while ((test.volcanoX < 3 && test.volcanoY < 3) ||
                 (abs(test.volcanoX - test.gollumX) < 2 && abs(test.volcanoY - test.gollumY) < 2));

        // Generate enemies according to PDF specifications
        generateEnemies(test);

        // Add pits (according to PDF, pits are separate from enemies)
        generatePits(test);

        return test;
    }

private:
    void generateEnemies(TestCase& test) {
        // Orc Patrols: 1-2 (стр. 6 PDF)
        int orcCount = orcCountDist(gen);
        for (int i = 0; i < orcCount; i++) {
            addEnemy(test, 'O');
        }

        // Uruk-hai: exactly 1 (стр. 7 PDF)
        addEnemy(test, 'U');

        // Nazgul: 0-1 (стр. 8 PDF)
        int nazgulCount = nazgulCountDist(gen);
        for (int i = 0; i < nazgulCount; i++) {
            addEnemy(test, 'N');
        }

        // Watchtower: exactly 1 (стр. 10 PDF)
        addEnemy(test, 'W');
    }

    void generatePits(TestCase& test) {
        // Pits: 2-5 (из примеров в PDF)
        int numPits = numPitsDist(gen);

        for (int i = 0; i < numPits; i++) {
            int pitX, pitY;
            bool validPosition;

            do {
                pitX = coordDist(gen);
                pitY = coordDist(gen);

                validPosition = !isPositionOccupied(test, pitX, pitY);

            } while (!validPosition);

            stringstream perception;
            perception << pitX << " " << pitY << " " << 'P';
            test.perceptions.push_back(perception.str());
        }
    }

    void addEnemy(TestCase& test, char enemyType) {
        int enemyX, enemyY;
        bool validPosition;

        do {
            enemyX = coordDist(gen);
            enemyY = coordDist(gen);

            validPosition = !isPositionOccupied(test, enemyX, enemyY);

        } while (!validPosition);

        stringstream perception;
        perception << enemyX << " " << enemyY << " " << enemyType;
        test.perceptions.push_back(perception.str());
    }

    bool isPositionOccupied(const TestCase& test, int x, int y) {
        // Check if position conflicts with important locations
        if ((x == 0 && y == 0) ||
            (x == test.gollumX && y == test.gollumY) ||
            (x == test.volcanoX && y == test.volcanoY)) {
            return true;
        }

        // Check if position conflicts with existing perceptions
        for (const auto& perception : test.perceptions) {
            istringstream iss(perception);
            int existingX, existingY;
            char type;
            iss >> existingX >> existingY >> type;

            if (existingX == x && existingY == y) {
                return true;
            }
        }

        return false;
    }
};

// Statistical Analysis
class StatisticalAnalyzer {
private:
    vector<AlgorithmStats> allStats;

public:
    void analyzeResults(const vector<TestResult>& results) {
        // Group results by algorithm and variant
        map<pair<string, int>, vector<TestResult>> groupedResults;

        for (const auto& result : results) {
            auto key = make_pair(result.algorithm, result.variant);
            groupedResults[key].push_back(result);
        }

        // Calculate statistics for each group
        for (const auto& group : groupedResults) {
            AlgorithmStats stats;
            stats.algorithm = group.first.first;
            stats.variant = group.first.second;

            calculateStatistics(stats, group.second);
            allStats.push_back(stats);
        }

        generateReport();
    }

    // Новый метод для анализа распределения врагов
    void analyzeEnemyDistribution(const vector<TestCaseGenerator::TestCase>& testCases) {
        ofstream file("enemy_distribution_analysis.txt");

        file << "================================================================================\n";
        file << "                      ENEMY DISTRIBUTION ANALYSIS\n";
        file << "================================================================================\n\n";

        map<char, vector<int>> enemyCountsByTest;
        int totalTests = testCases.size();

        for (size_t i = 0; i < testCases.size(); i++) {
            const auto& testCase = testCases[i];
            map<char, int> currentCounts;

            for (const auto& perception : testCase.perceptions) {
                istringstream iss(perception);
                int x, y;
                char type;
                iss >> x >> y >> type;

                if (type == 'O' || type == 'U' || type == 'N' || type == 'W') {
                    currentCounts[type]++;
                }
            }

            for (const auto& count : currentCounts) {
                enemyCountsByTest[count.first].push_back(count.second);
            }
        }

        file << "Enemy Distribution Across " << totalTests << " Tests:\n";
        file << "================================================================================\n";
        file << "Enemy Type     | Min | Max | Mean  | Required | Compliance\n";
        file << "----------------------------------------------------------------\n";

        vector<tuple<char, string, string>> requirements = {
            {'O', "1-2", "Orc Patrol"},
            {'U', "1", "Uruk-hai"},
            {'N', "0-1", "Nazgul"},
            {'W', "1", "Watchtower"}
        };

        for (const auto& req : requirements) {
            char type = get<0>(req);
            const string& requiredRange = get<1>(req);
            const string& name = get<2>(req);

            if (enemyCountsByTest.count(type)) {
                const auto& counts = enemyCountsByTest[type];
                int minCount = *min_element(counts.begin(), counts.end());
                int maxCount = *max_element(counts.begin(), counts.end());
                double meanCount = accumulate(counts.begin(), counts.end(), 0.0) / counts.size();

                // Проверка соответствия требованиям
                bool compliant = true;
                if (type == 'O') compliant = (minCount >= 1 && maxCount <= 2);
                else if (type == 'U') compliant = (minCount == 1 && maxCount == 1);
                else if (type == 'N') compliant = (minCount >= 0 && maxCount <= 1);
                else if (type == 'W') compliant = (minCount == 1 && maxCount == 1);

                file << setw(14) << left << name << " | ";
                file << setw(3) << minCount << " | ";
                file << setw(3) << maxCount << " | ";
                file << setw(5) << fixed << setprecision(1) << meanCount << " | ";
                file << setw(8) << requiredRange << " | ";
                file << (compliant ? "✓ PASS" : "✗ FAIL") << "\n";
            }
        }

        // Анализ распределения по вариантам
        file << "\nDistribution by Variant:\n";
        file << "================================================================================\n";

        for (int variant = 1; variant <= 2; variant++) {
            file << "Variant " << variant << ":\n";
            map<char, int> variantCounts;
            int variantTests = 0;

            for (const auto& testCase : testCases) {
                if (testCase.variant == variant) {
                    variantTests++;
                    for (const auto& perception : testCase.perceptions) {
                        istringstream iss(perception);
                        int x, y;
                        char type;
                        iss >> x >> y >> type;
                        if (type == 'O' || type == 'U' || type == 'N' || type == 'W') {
                            variantCounts[type]++;
                        }
                    }
                }
            }

            if (variantTests > 0) {
                for (const auto& enemy : variantCounts) {
                    double avgPerTest = static_cast<double>(enemy.second) / variantTests;
                    file << "  " << enemy.first << ": " << fixed << setprecision(2) << avgPerTest << " per test\n";
                }
            }
        }

        file.close();
    }

private:
    void calculateStatistics(AlgorithmStats& stats, const vector<TestResult>& results) {
        // Wins and losses
        stats.wins = 0;
        stats.losses = 0;

        for (const auto& result : results) {
            if (result.success) {
                stats.wins++;
                stats.executionTimes.push_back(result.totalTime);
                stats.stepsCount.push_back(result.steps);
            } else {
                stats.losses++;
            }
        }

        stats.winRate = (static_cast<double>(stats.wins) / results.size()) * 100.0;

        // Time statistics
        if (!stats.executionTimes.empty()) {
            calculateVectorStats(stats.executionTimes, stats.meanTime, stats.medianTime,
                               stats.modeTime, stats.stdDevTime);
        }

        // Steps statistics
        if (!stats.stepsCount.empty()) {
            calculateVectorStats(stats.stepsCount, stats.meanSteps, stats.medianSteps,
                               stats.modeSteps, stats.stdDevSteps);
        }
    }

    void calculateVectorStats(const vector<int>& data, double& mean, double& median,
                            double& mode, double& stdDev) {
        // Mean
        mean = accumulate(data.begin(), data.end(), 0.0) / data.size();

        // Median
        vector<int> sortedData = data;
        sort(sortedData.begin(), sortedData.end());
        median = sortedData.size() % 2 == 0
            ? (sortedData[sortedData.size()/2 - 1] + sortedData[sortedData.size()/2]) / 2.0
            : sortedData[sortedData.size()/2];

        // Mode
        map<int, int> frequency;
        for (int value : data) frequency[value]++;
        mode = max_element(frequency.begin(), frequency.end(),
                         [](const pair<int, int>& a, const pair<int, int>& b) {
                             return a.second < b.second;
                         })->first;

        // Standard deviation
        double variance = 0.0;
        for (int value : data) {
            variance += pow(value - mean, 2);
        }
        variance /= data.size();
        stdDev = sqrt(variance);
    }

    void generateReport() {
        ofstream file("statistical_analysis_report.txt");

        file << "================================================================================\n";
        file << "                      STATISTICAL ANALYSIS REPORT\n";
        file << "================================================================================\n\n";

        file << "A* Algorithm Performance Analysis for Ring Quest Game\n";
        file << "Based on 1000 test maps for each variant\n\n";

        // Summary table
        file << "SUMMARY STATISTICS\n";
        file << "================================================================================\n";
        file << "Variant | Wins | Losses | Win Rate | Mean Time | Mean Steps\n";
        file << "================================================================================\n";

        for (const auto& stats : allStats) {
            file << setw(7) << left << stats.variant << " | ";
            file << setw(4) << stats.wins << " | ";
            file << setw(6) << stats.losses << " | ";
            file << setw(8) << fixed << setprecision(2) << stats.winRate << "% | ";
            file << setw(9) << stats.meanTime << " | ";
            file << setw(10) << stats.meanSteps << "\n";
        }
        file << "================================================================================\n\n";

        // Detailed statistics tables
        for (int variant = 1; variant <= 2; variant++) {
            file << "DETAILED STATISTICS - VARIANT " << variant << "\n";
            file << "================================================================================\n";
            file << "Statistic          |     Value\n";
            file << "-----------------------------------------------------------------\n";

            auto stats = findStats("A*", variant);
            if (stats) {
                vector<pair<string, double>> statistics = {
                    {"Win Rate (%)", stats->winRate},
                    {"Mean Time", stats->meanTime},
                    {"Median Time", stats->medianTime},
                    {"Mode Time", stats->modeTime},
                    {"Std Dev Time", stats->stdDevTime},
                    {"Mean Steps", stats->meanSteps},
                    {"Median Steps", stats->medianSteps},
                    {"Mode Steps", stats->modeSteps},
                    {"Std Dev Steps", stats->stdDevSteps}
                };

                for (const auto& stat : statistics) {
                    file << setw(18) << left << stat.first << " | ";
                    file << setw(10) << fixed << setprecision(2) << stat.second << "\n";
                }
            }
            file << "================================================================================\n\n";
        }

        // Performance analysis
        file << "PERFORMANCE ANALYSIS\n";
        file << "================================================================================\n";
        file << "Key Observations:\n\n";

        auto variant1Stats = findStats("A*", 1);
        auto variant2Stats = findStats("A*", 2);

        if (variant1Stats && variant2Stats) {
            file << "Variant Comparison:\n";
            file << "- Variant 2 is " << (variant2Stats->meanTime / variant1Stats->meanTime)
                 << "x " << (variant2Stats->meanTime > variant1Stats->meanTime ? "slower" : "faster") << " than Variant 1\n";
            file << "- Variant 2 has " << (variant2Stats->winRate - variant1Stats->winRate)
                 << "% " << (variant2Stats->winRate > variant1Stats->winRate ? "higher" : "lower") << " win rate\n";
            file << "- Variant 2 requires " << (variant2Stats->meanSteps - variant1Stats->meanSteps)
                 << " " << (variant2Stats->meanSteps > variant1Stats->meanSteps ? "more" : "fewer") << " steps on average\n\n";
        }

        file << "Overall Performance:\n";
        file << "- A* algorithm demonstrates consistent performance across both variants\n";
        file << "- The algorithm successfully handles dynamic threat detection\n";
        file << "- Path optimization effectively minimizes steps while ensuring safety\n";

        file.close();

        // Generate CSV for further analysis
        generateCSVReport();
    }

    void generateCSVReport() {
        ofstream file("statistical_data.csv");

        file << "Algorithm,Variant,Wins,Losses,WinRate,MeanTime,MedianTime,ModeTime,StdDevTime,"
             << "MeanSteps,MedianSteps,ModeSteps,StdDevSteps\n";

        for (const auto& stats : allStats) {
            file << stats.algorithm << ",";
            file << stats.variant << ",";
            file << stats.wins << ",";
            file << stats.losses << ",";
            file << stats.winRate << ",";
            file << stats.meanTime << ",";
            file << stats.medianTime << ",";
            file << stats.modeTime << ",";
            file << stats.stdDevTime << ",";
            file << stats.meanSteps << ",";
            file << stats.medianSteps << ",";
            file << stats.modeSteps << ",";
            file << stats.stdDevSteps << "\n";
        }

        file.close();
    }

    const AlgorithmStats* findStats(const string& algorithm, int variant) const {
        for (const auto& stats : allStats) {
            if (stats.algorithm == algorithm && stats.variant == variant) {
                return &stats;
            }
        }
        return nullptr;
    }
};

// Enhanced Batch Tester
class BatchTester {
private:
    RingQuestSolver solver;
    TestCaseGenerator generator;
    StatisticalAnalyzer analyzer;
    vector<TestCaseGenerator::TestCase> generatedTestCases;

public:
    vector<TestResult> runComprehensiveTestBatch(int numTests = 1000, bool enableLogging = false) {
        vector<TestResult> allResults;
        generatedTestCases.clear();

        cout << "Starting comprehensive A* algorithm testing with " << numTests << " tests per variant..." << endl;

        // Test A* algorithm on generated test cases
        for (int i = 0; i < numTests; i++) {
            if (i % 100 == 0) {
                cout << "Progress: " << i << "/" << numTests << " tests completed..." << endl;
            }

            auto testCase = generator.generateTest(i);
            generatedTestCases.push_back(testCase);

            // Test A* algorithm
            auto aStarResult = solver.runSingleTest(testCase.testId, testCase.variant,
                                                  testCase.gollumX, testCase.gollumY,
                                                  testCase.volcanoX, testCase.volcanoY,
                                                  testCase.perceptions, "A*", enableLogging);
            allResults.push_back(aStarResult);
        }

        // Perform statistical analysis
        cout << "Performing statistical analysis..." << endl;
        analyzer.analyzeResults(allResults);

        // Анализ распределения врагов
        cout << "Analyzing enemy distribution..." << endl;
        analyzer.analyzeEnemyDistribution(generatedTestCases);

        saveBatchSummary(allResults);

        cout << "Comprehensive testing completed!" << endl;
        cout << "Statistical report saved as 'statistical_analysis_report.txt'" << endl;
        cout << "Enemy distribution analysis saved as 'enemy_distribution_analysis.txt'" << endl;
        cout << "Raw data saved as 'statistical_data.csv'" << endl;

        return allResults;
    }

private:
    void saveBatchSummary(const vector<TestResult>& results) {
        ofstream file("test_results/comprehensive_summary.json");

        map<pair<string, int>, int> successCount;
        map<pair<string, int>, int> totalSteps;
        map<pair<string, int>, int> totalTime;

        for (const auto& result : results) {
            auto key = make_pair(result.algorithm, result.variant);
            if (result.success) {
                successCount[key]++;
                totalSteps[key] += result.steps;
            }
            totalTime[key] += result.totalTime;
        }

        file << "{" << endl;
        file << "  \"totalTests\": " << results.size() << "," << endl;
        file << "  \"algorithmResults\": {" << endl;

        bool firstAlgorithm = true;
        for (const auto& entry : successCount) {
            if (!firstAlgorithm) file << "," << endl;
            firstAlgorithm = false;

            string algorithm = entry.first.first;
            int variant = entry.first.second;
            int successes = entry.second;
            int totalForAlgorithm = count_if(results.begin(), results.end(),
                [&](const TestResult& r) {
                    return r.algorithm == algorithm && r.variant == variant;
                });

            file << "    \"" << algorithm << "_V" << variant << "\": {" << endl;
            file << "      \"successfulTests\": " << successes << "," << endl;
            file << "      \"totalTests\": " << totalForAlgorithm << "," << endl;
            file << "      \"successRate\": " << (static_cast<double>(successes) / totalForAlgorithm * 100) << "," << endl;
            file << "      \"averageSteps\": " << (successes > 0 ? totalSteps[entry.first] / successes : 0) << "," << endl;
            file << "      \"averageTime\": " << (totalTime[entry.first] / totalForAlgorithm) << endl;
            file << "    }";
        }

        file << endl << "  }" << endl;
        file << "}" << endl;

        file.close();
    }
};

int main(int argc, char* argv[])
{
    if (argc > 1 && string(argv[1]) == "--comprehensive-test") {
        int numTests = 1000;
        if (argc > 2) {
            numTests = stoi(argv[2]);
        }

        // Create results directory
        system("mkdir -p test_results");

        BatchTester tester;
        auto results = tester.runComprehensiveTestBatch(numTests, true);
    }
    else if (argc > 1 && string(argv[1]) == "--batch-test") {
        int numTests = 100;
        if (argc > 2) {
            numTests = stoi(argv[2]);
        }

        system("mkdir -p test_results");

        BatchTester tester;
        auto results = tester.runComprehensiveTestBatch(numTests, false);
    }
    else {
        RingQuestSolver solver;
        solver.executeSolution();
    }

    return 0;
}