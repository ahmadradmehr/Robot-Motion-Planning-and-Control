#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cfloat>
#include <limits>
#include <random>
#include <utility>

using namespace std;

// A function to check whether there is a collision with obstacles
// if an edge is created between first_node and second_node
bool collisionFree (vector<vector<double>> obs, vector<double> first_node, vector<double> second_node) {
    
    // line definition ax + by + c = 0
    double a {0}, b {1}, c {0}, dis {0};
    a = -(first_node.at(1) - second_node.at(1)) / (first_node.at(0) - second_node.at(0));
    c = -a * first_node.at(0) - first_node.at(1);
    for (int i {0}; i < obs.size(); i++) {
        dis = fabs(a * obs.at(i).at(0) + b * obs.at(i).at(1) + c) / sqrt(pow(a, 2) + pow(b, 2));
        if (dis <= obs.at(i).at(2) / 2)
            return false;
    }
    return true;
}

bool AstarAlgorithm(ofstream &file, vector<vector<double>> nodes, vector<vector<double>> edges) {
    
    // Generating heuristics vector to keep heuristic costs of all nodes
    vector<double> heuristic (nodes.size(), 0.0);
    for (int i {0}; i < nodes.size(); i++) {
        heuristic.at(i) = (double)nodes.at(i).at(2);
    }
    
    // Creating a cost matrix to keep the cost of edges
    vector<vector<double>> cost (heuristic.size(), vector<double> (heuristic.size(), -1));
    
    for (int i {0}; i < edges.size(); i++) {
        cost.at(edges.at(i).at(0)).at(edges.at(i).at(1)) = edges.at(i).at(2);
        cost.at(edges.at(i).at(1)).at(edges.at(i).at(0)) = edges.at(i).at(2);
    }
    
    // Defining Open and Close list of nodes
    vector<pair<double, int>> open {{0.0, 1}};
    vector<int> close;
    
    // Defining past costs of all nodes
    vector<double> past_cost (nodes.size(), DBL_MAX);
    past_cost.at(0) = 0; // cost of first node
    
    // Defining parent of all nodes
    vector<int> parent (nodes.size(), -1);
    
    // Defining a path vector to keep the result!
    vector<int> generated_path;
    
    bool found = false;
    // Starting the A* search algorithm
    while (!open.empty()) {
        pair<double, int> current = open.front();
        open.erase(open.begin());
        close.push_back(current.second);
        if (current.second == nodes.size()) {
            int node = current.second;
            while (node != 1) {
                generated_path.push_back(node);
                node = parent.at(node-1);
            }
            generated_path.push_back(node);
            reverse(generated_path.begin(), generated_path.end());
            file.open("path.csv");
            for (int i {0}; i < generated_path.size()-1; i++) {
                file << generated_path.at(i) << ',';
            }
            file << generated_path.back() << endl;
            file.close();
            found = true;
            break;
        }
        for (int i {0}; i < nodes.size(); i++) {
            if (cost.at(current.second-1).at(i) == -1 ||
            find(close.begin(), close.end(), i+1) != close.end())
                continue;
            double tentative_past_cost = past_cost.at(current.second-1) + cost.at(current.second-1).at(i);
            if (tentative_past_cost < past_cost.at(i)) {
                past_cost.at(i) = tentative_past_cost;
                parent.at(i) = current.second;
                double est_total_cost = past_cost.at(i) + heuristic.at(i);
                open.push_back({est_total_cost, i+1});
                sort(open.begin(), open.end(), [] (pair<double, int> &p1, pair<double, int> &p2) {
                    return p1.first < p2.first;
                });
            }
        }
    }
    if (!found) {
        generated_path.push_back(1);
        file.open("path.csv");
        file << 1 << endl;
        file.close();
        return false;
    }
    return true;
}

int main() {
    // Reading data from the obstacles.csv file
    ifstream obstacles ("obstacles.csv");
    
    // Creating a vector to store obstacles data
    vector<vector<double>> obs;
    
    while (obstacles.good()) {
        string x_coor {""}, y_coor {""}, radius {""};
        getline(obstacles, x_coor, ',');
        if (x_coor.empty())
            break;
        if (x_coor.front() == '#') {
            obstacles.ignore(numeric_limits<streamsize>::max(), '\n');
            continue;
        }
        getline(obstacles, y_coor, ',');
        getline(obstacles, radius, '\n');
        obs.push_back({stod(x_coor), stod(y_coor), stod(radius)});
    }
    
    // Creating a path to store the generated files
    ofstream file;
    
    if (!obstacles.is_open())
        cout << "obstacles.csv is not open!" << endl;
    
    // Creating a vector to store nodes coordinates
    vector<vector<double>> nodes;
    
    // Defining x_goal;
    pair<double, double> x_goal {0.5, 0.5};
    
    nodes.emplace_back(vector<double> {-0.5, -0.5, sqrt(pow(-0.5 - x_goal.first, 2) + pow(-0.5 - x_goal.second, 2))});
    
    // Creating a vector to store edges
    vector<vector<double>> edges;
    
    // Defining number of nodes
    int N = 100;
    
    // Defining variables for uniform random sampling
    default_random_engine generator;
    uniform_real_distribution<double> distribution(-0.5, 0.5);
    
    for (int i {0}; i < N-2; i++) {
        // Sampling
        pair<double, double> x_sample {distribution(generator), distribution(generator)};
        nodes.emplace_back(vector<double> {x_sample.first, x_sample.second,
            sqrt(pow(x_sample.first - x_goal.first, 2) + pow(x_sample.second - x_goal.second, 2))});
    }
    nodes.emplace_back(vector<double> {x_goal.first, x_goal.second, 0});
    
    // Defining number of closest neighbors of each node to search for
    int k = 11;
    
    // Starting looping through PRM Algorithm
    for (int i {0}; i < N; i++) {
        
        // Finding k nearest nodes to node[i]
        vector<double> min_dist;
        vector<int> nearest_node_ids;
        for (int j {0}; j < N; j++) {
            if (i == j)
                continue;
            double neighbor_dist = sqrt(pow(nodes.at(i).at(0) - nodes.at(j).at(0), 2) + pow(nodes.at(i).at(1) - nodes.at(j).at(1), 2));
            if (nearest_node_ids.size() < k) {
                min_dist.push_back(neighbor_dist);
                nearest_node_ids.push_back(j);
            } else {
                for (int l {0}; l < k; l++) {
                    if (neighbor_dist < min_dist.at(l)) {
                        min_dist.at(l) = neighbor_dist;
                        nearest_node_ids.at(l) = j;
                        break;
                    }
                }
            }
        }
        
        for (int j {0}; j < k; j++) {
            // Checking if there is already an edge between i and j
            bool edge_exists = false;
            for (int l {0}; l < edges.size(); l++) {
                if (edges.at(l).at(0) == i && edges.at(l).at(1) == nearest_node_ids.at(j) ||
                edges.at(l).at(0) == nearest_node_ids.at(j) && edges.at(l).at(1) == i)
                    edge_exists = true;
            }
            
            // Checking if the motion from x_nearest to x_new if collision-free
            if (collisionFree(obs, nodes.at(i), nodes.at(nearest_node_ids.at(j))) && !edge_exists) {
                
                // if it is collision-free, create an edge
                edges.emplace_back(vector<double> {(double)i, (double)nearest_node_ids.at(j), min_dist.at(j)});
            }
        }
    }
    
    // Writing generated nodes
    file.open("nodes.csv");
    for (int i {0}; i < nodes.size(); i++) {
        file << i+1 << ',' << nodes.at(i).at(0) << ','
            << nodes.at(i).at(1) << ',' << nodes.at(i).at(2) << endl;
    }
    file.close();
    
    // Writing generated edges
    file.open("edges.csv");
    for (int i {0}; i < edges.size(); i++) {
        file << (int)edges.at(i).at(0)+1 << ',' << (int)edges.at(i).at(1)+1 << ',' << edges.at(i).at(2) << endl;
    }
    file.close();
    
    bool path_found = AstarAlgorithm(file, nodes, edges);
    
    if (path_found)
        cout << "nodes, edges, and path created" << endl;
    else
        cout << "No path found!" << endl;

    return 0;
}