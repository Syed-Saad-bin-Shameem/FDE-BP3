#ifndef FDE20_BONUSPROJECT_3_KNN_HPP
#define FDE20_BONUSPROJECT_3_KNN_HPP

#include <queue>
#include <map>
#include "Matrix.hpp"
#include "limits"
#include <cfloat>
#include "algorithm"

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
    std::multimap<double , unsigned > node_dist_MMap;
    pq.emplace(start, 0.0);
    dist[start] = 0.0;
    while (!pq.empty()){
            unsigned v = pq.top().column;
            double d = pq.top().weight;
            pq.pop();
            //if (!visited[v]){
                //visited[v] = true;
                if (d <= dist[v]){
                    for (auto &i: m.getNeighbors(v)){
                        if (!visited[i.column]){
                            visited[i.column] = true;
                            unsigned v2 = i.column;
                            double cost = i.weight;
                            if (dist[v2] > dist[v] + cost){
                                dist[v2] = dist[v] + cost;
                                node_dist_MMap.insert(std::pair(dist[v2], v2));
                                pq.emplace(v2, dist[v2]);
                        }
                        /*unsigned v2 = i.column;
                        double cost = i.weight;
                        if (dist[v2] > dist[v] + cost){
                            dist[v2] = dist[v] + cost;
                            node_dist_MMap.insert(std::pair(dist[v2], v2));
                            pq.emplace(v2, dist[v2]);*/
                        }
                    }
                }
            //}
    }
    unsigned j = 0;
    for (auto & i : node_dist_MMap){
        if (j >= k){
            break;
        }
        else{
            if (std::find(result.begin(), result.end(), Entry(i.second, i.first)) != result.end()){
                continue;
            }
            else{
                result.emplace_back(i.second, i.first);
                j += 1;
            }
        }
        //result.emplace_back(i.second, i.first);
        //j += 1;
    }
    return result;
}

//---------------------------------------------------------------------------

#endif // FDE20_BONUSPROJECT_3_KNN_HPP
