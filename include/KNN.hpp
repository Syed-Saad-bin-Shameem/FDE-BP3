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

    // ToDo implement your solution here
    // Custom comparator function for use in STL priority queue
    auto compare = [](Entry lhs, Entry rhs){
        return lhs.weight < rhs.weight; // replace < with >
    };
    struct myComp {
        constexpr bool operator()(
                Entry const& a,
                Entry const& b)
        const noexcept
        {
            return a.weight < b.weight;
        }
    };
    //std::priority_queue<Entry, std::vector<Entry>, decltype(compare)> pq(compare);

    // Testing custom comparator only
    //std::priority_queue<Entry, std::vector<Entry>, std::greater<std::vector<Entry>>> pq;
    //std::priority_queue<Entry, std::vector<Entry>, std::less<std::vector<Entry>>> pq;
    //std::priority_queue<Entry, std::vector<Entry>, std::less<Entry>> pq;
    //std::priority_queue<Entry, std::vector<Entry>, std::less<Entry>> pq;
    //std::priority_queue<Entry, std::vector<Entry>, myComp> pq;
    std::priority_queue<Entry> pq;
    //std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    //std::priority_queue<Entry, std::vector<Entry>> pq;
    // Testing stop

    //std::vector<double> dist(m.getRowCount(), INFINITY);
    //std::vector<double> dist(m.getRowCount(), std::numeric_limits<double>::max());
    std::vector<double> dist(m.getRowCount(), DBL_MAX);
    std::map<double , unsigned > node_dist_Map;
    pq.emplace(start, 0);
    //Entry start_node = Entry(start, 0);
    //Entry start_node = Entry(0, start);
    //pq.push(start_node);
    //pq.push(Entry(start, 0));
    /*dist[start] = 0;
    while (!pq.empty()){
        auto temp_node = pq.top();
        unsigned u = temp_node.column;
        pq.pop();
        for (auto &i: m.getNeighbors(temp_node.column)){
            unsigned v = i.column;
            double weight = i.weight;
            if (dist[v] > dist[u] + weight){
                dist[v] = dist[u] + weight;
                //pq.emplace(v, dist[v]);
                Entry temp = Entry(v, dist[v]);
                pq.push(temp);
            }
        }
    }*/
    dist[start] = 0;
    while (!pq.empty()){
        unsigned v = pq.top().column;
        double d = pq.top().weight;
        pq.pop();
        if (d <= dist[v]){
            for (auto &i: m.getNeighbors(v)){
                unsigned v2 = i.column;
                double cost = i.weight;
                if (dist[v2] > dist[v] + cost){
                    dist[v2] = dist[v] + cost;
                    node_dist_Map[dist[v2]] = v2;
                    pq.emplace(v2, dist[v2]);
                    //Entry temp = Entry(v2, dist[v2]);
                    //Entry temp = Entry(dist[v2], v2);
                    //pq.push(temp);
                    pq.top();
                }
            }
        }
    }
    std::sort_heap(dist.begin(), dist.end());
    for (int j=0; j<k; j++){
        //printf("Dist is " , dist[j]);
        //printf("Dist from map is " , node_dist_Map[dist[j+1]]);
        //Entry temp_node_2 = Entry(pq.top().weight, pq.top().column);
        //temp_node_2.weight
        //result.push_back()
        Entry temp = Entry(node_dist_Map[dist[j+1]], dist[j+1]);
        result.push_back(temp);
        //result.push_back(pq.top());
        pq.pop();
    }
    return result;
}

//---------------------------------------------------------------------------

#endif // FDE20_BONUSPROJECT_3_KNN_HPP
