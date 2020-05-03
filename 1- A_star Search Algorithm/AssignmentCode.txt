#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cfloat>
#include <limits>

using namespace std;

int main() {
    // Reading data from the .CSV files
    ifstream nodes ("nodes.csv");
    ifstream edges ("edges.csv");
    
    // Creating a path file to store the generated path
    ofstream file;
    
    if (!nodes.is_open())
        cout << "nodes.csv is not open!" << endl;
    if (!edges.is_open())
        cout << "edges.csv is not open!" << endl;
    
    // Generating heuristics vector to keep heuristic costs of all nodes
    vector<double> heuristic;
    
    while (nodes.good()) {
        string temp {""};
        getline(nodes, temp, ',');
        if (temp.empty())
            break;
        if (temp.front() == '#') {
            nodes.ignore(numeric_limits<streamsize>::max(), '\n');
            continue;
        }
        getline(nodes, temp, ',');
        getline(nodes, temp, ',');
        getline(nodes, temp, '\n');
        heuristic.push_back(stod(temp));
    }
    
    // Displaying heuristic costs for all nodes
    cout << "Heuristic Costs:\n" << endl;
    for (int i {0}; i < heuristic.size(); i++) {
        cout << left << setw(5) << i+1 << setw(10) << heuristic.at(i) << endl;
    }
    cout << "--------------------------------\n" << endl;
    
    // Creating a cost matrix to keep the cost of edges
    vector<vector<double>> cost (heuristic.size(), vector<double> (heuristic.size(), -1));
    
    while (edges.good()) {
        string node1, node2, c;
        getline(edges, node1, ',');
        if (node1.empty())
            break;
        if (node1.front() == '#') {
            edges.ignore(numeric_limits<streamsize>::max(), '\n');
            continue;
        }
        int n1, n2;
        n1 = stoi(node1);
        getline(edges, node2, ',');
        n2 = stoi(node2);
        getline(edges, c);
        cost[n1-1][n2-1] = stod(c);
        cost[n2-1][n1-1] = stod(c);
    }
    
    // Displaying the cost of edges
    cout << "Edges Cost:\n" << endl;
    cout << setw(7) << "";
    for (int i {0}; i < heuristic.size(); i++) {
        cout << setw(7) << i+1;
    }
    cout << endl;
    for (int i {0}; i < cost.size(); i++) {
        cout << setw(7) << i+1;
        for (int j {0}; j < cost.size(); j++) {
            cout << setw(7) << cost.at(i).at(j);
        }
        cout << endl;
    }
    cout << "--------------------------------\n" << endl;
    
    // Defining Open and Close list of nodes
    vector<pair<double, int>> open {{0.0, 1}};
    vector<int> close;
    
    // Defining past costs of all nodes
    vector<double> past_cost (heuristic.size(), DBL_MAX);
    past_cost.at(0) = 0; // cost of first node
    
    // Defining parent of all nodes
    vector<int> parent (heuristic.size(), -1);
    
    // Defining a path vector to keep the result!
    vector<int> generated_path;
    
    bool found = false;
    // Starting the A* search algorithm
    while (!open.empty()) {
        pair<double, int> current = open.front();
        open.erase(open.begin());
        close.push_back(current.second);
        if (current.second == heuristic.size()) {
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
        for (int i {0}; i < heuristic.size(); i++) {
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
    }
    
    // Printing the parents vector
    cout << "Parents:\n" << endl;
    for (int i {0}; i < parent.size(); i++) {
        cout << parent.at(i) << " ";
    }
    cout << endl << endl;
    
    // Printing the generated path
    cout << "Generated Path:\n" << endl;
    for (int i {0}; i < generated_path.size(); i++) {
        cout << generated_path.at(i) << " ";
    }
    cout << endl;
    
    return 0;
}