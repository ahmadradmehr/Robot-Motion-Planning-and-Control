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
#include <math.h>

using namespace std;

// A function to check whether there is a collision with obstacles
// if an edge is created between first_node and second_node
bool collisionFree (vector<vector<double>> obs, vector<double> current_node, pair<double, double> sample_node) {
    // line definition ax + by + c = 0
    double a {0}, b {1}, c {0}, dis {0};
    a = -(current_node.at(1) - sample_node.second) / (current_node.at(0) - sample_node.first);
    c = -a * current_node.at(0) - current_node.at(1);
    for (int i {0}; i < obs.size(); i++) {
        dis = fabs(a * obs.at(i).at(0) + b * obs.at(i).at(1) + c) / sqrt(pow(a, 2) + pow(b, 2));
        if (dis <= obs.at(i).at(2) / 2)
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
    
    // Defining maximum tree size
    int max_tree_size = 1000;
    
    // Defining variables for uniform random sampling
    default_random_engine generator;
    uniform_real_distribution<double> distribution(-0.5, 0.5);
    
    // Storing parents of nodes
    vector<int> parents {-1};
    
    // Starting looping through RRT Algorithm
    while (nodes.size() < max_tree_size) {
        
        // Sampling
        pair<double, double> x_sample {distribution(generator), distribution(generator)};
        
        // Finding nearest node to x_sample
        double min_dist = DBL_MAX;
        int nearest_node_id {0};
        for (int i {0}; i < nodes.size(); i++) {
            double dist = sqrt(pow(nodes.at(i).at(0) - x_sample.first, 2) + pow(nodes.at(i).at(1) - x_sample.second, 2));
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }
        
        // employing the straight-line local planner
        double d = 0.1;
        pair<double, double> x_new;
        if (sqrt(pow(nodes.at(nearest_node_id).at(0) - x_goal.first, 2) + pow(nodes.at(nearest_node_id).at(1) - x_goal.second, 2)) <= d)
            x_new = x_sample;
        else {
            double slope = (nodes.at(nearest_node_id).at(1) - x_sample.second) / (nodes.at(nearest_node_id).at(0) - x_sample.first);
            x_new.first = d * sqrt(1 / (1 + pow(slope, 2))) + nodes.at(nearest_node_id).at(0);
            x_new.second = d * sqrt(pow(slope, 2) / (1 + pow(slope, 2))) + nodes.at(nearest_node_id).at(1);
        }
        
        // Checking if the motion from x_nearest to x_new if collision-free
        if (collisionFree(obs, nodes.at(nearest_node_id), x_new)) {
            // if it is collision-free, add it to the nodes and create an edge
            nodes.emplace_back(vector<double> {x_new.first, x_new.second,
                sqrt(pow(x_new.first - x_goal.first, 2) + pow(x_new.second - x_goal.second, 2))});
            edges.emplace_back(vector<double> {(double)nearest_node_id, (double)nodes.size()-1, 
                sqrt(pow(x_new.first - nodes.at(nearest_node_id).at(0), 2) + pow(x_new.second - nodes.at(nearest_node_id).at(1), 2))});
            parents.push_back(nearest_node_id);
            // Check if x_new is in x_goal
            if (sqrt(pow(x_new.first - x_goal.first, 2) + pow(x_new.second - x_goal.second, 2)) < 0.1 && 
                collisionFree(obs, vector<double> {x_goal.first, x_goal.second}, x_new)) {
                nodes.emplace_back(vector<double> {x_goal.first, x_goal.second, 0.0});
                edges.emplace_back(vector<double> {(double)nodes.size()-2, (double)nodes.size()-1, 
                    sqrt(pow(x_new.first - x_goal.first, 2) + pow(x_new.second - x_goal.second, 2))});
                parents.push_back(nodes.size()-2);
                break;
            }
        }
    }
    
    if (nodes.back().at(0) == x_goal.first &&  nodes.back().at(1) == x_goal.second) {
        
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
        
        // Writing path.csv file
        int n = nodes.size()-1;
        vector<int> p;
        p.push_back(nodes.size()-1);
        while (parents.at(n) != 0) {
            p.push_back(parents.at(n));
            n = parents.at(n);
        }
        p.push_back(0);
        reverse(p.begin(), p.end());
        
        file.open("path.csv");
        for (int i {0}; i < p.size()-1; i++) {
            file << p.at(i)+1 << ',';
        }
        file << p.at(p.size()-1)+1 << endl;
        file.close();
        
        cout << "nodes, edges, and path created" << endl;
        
    } else {
        cout << "No Path Found" << endl;
    }
    
    return 0;
}