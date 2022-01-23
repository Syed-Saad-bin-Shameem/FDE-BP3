#ifndef FDE20_BONUSPROJECT_3_KNN_HPP
#define FDE20_BONUSPROJECT_3_KNN_HPP

#include <queue>
#include <map>
#include "Matrix.hpp"
#include "limits"
#include "float.h"

//---------------------------------------------------------------------------
/// Find the top k neighbors for the node start. The directed graph is stored in
/// matrix m and you can use getNeighbors(node) to get all neighbors of a node.
/// A more detailed description is provided in Matrix.hpp.
/// The function should return the k nearest neighbors in sorted order as vector
/// of Matrix::Entry, where Entry->column is the neighbor node and Entry->weight
/// is the cost to reach the node from the start node.
std::vector<Matrix::Entry> getKNN(const Matrix &m, unsigned start, unsigned k) {

    using Entry = Matrix::Entry;
    std::vector<Entry> result;
    result.reserve(k);

    std::map<unsigned , bool> visited;
    std::priority_queue<Entry> pq;
    std::vector<double> dist(m.getRowCount(), DBL_MAX);
    std::map<double , unsigned > node_dist_Map;
    pq.emplace(start, 0);
    dist[start] = 0;
    while (!pq.empty()){
        //if (pq.top().weight != dist[pq.top().column]) continue;
            unsigned v = pq.top().column;
            double d = pq.top().weight;
            pq.pop();
            if (visited[v] != true){
            visited[v] = true;
            if (d <= dist[v]){
                for (auto &i: m.getNeighbors(v)){
                    unsigned v2 = i.column;
                    double cost = i.weight;
                    if (dist[v2] > dist[v] + cost && cost < DBL_MAX){
                        dist[v2] = dist[v] + cost;
                        node_dist_Map[dist[v2]] = v2;
                        pq.emplace(v2, dist[v2]);
                        pq.top();
                    }
                }
            }
        }
    }
    std::sort_heap(dist.begin(), dist.end());
    for (int j=1; j<=k; j++){
        if (dist[j] < DBL_MAX){
            Entry temp = Entry(node_dist_Map[dist[j]], dist[j]);
            result.push_back(temp);
        }
        pq.pop();
    }
    return result;
}

//---------------------------------------------------------------------------

#endif // FDE20_BONUSPROJECT_3_KNN_HPP
