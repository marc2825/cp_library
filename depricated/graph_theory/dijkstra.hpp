/*
★ Name        ： Dijkstra (by priority queue)
 ■ Snipet      : Yes

 ■ Arguments   : 
 ■ Return      : 
 ■ Complexity  : 

 ■ Description : 
 

 ■ Usage       : 


 ■ Verify      : https://atcoder.jp/contests/abc035/submissions/44850919
 ■ References  : https://ei1333.github.io/library/graph/shortest-path/dijkstra.hpp

 ■ TODO        : graph_template とどう共存させるか（あっちは部分問題での超典型処理を瞬殺するためのもので柔軟性は微妙なので、スニペット的でいいかも）
*/


#pragma once
#include <vector>
#include <queue>
#include <limits>

using namespace std;


// Dijkstra (basic form)
// 0-indexed, returns the minimum distance of all points on the graph from the starting point
// Edge構造体やdijkstra関数の中身などを適宜弄る
template< typename T = long long >
struct Edge{
    int to;
    T cost;

    Edge() = default;
    Edge(int to, T cost = 1) : to(to), cost(cost) {}
};

template< typename T = long long >
using Graph = vector<vector<Edge> >;

template <typename T>
vector< T > dijkstra(const Graph< T > &G, int start = 0) {
  using P = pair<T, int>; // (cost, idx)
  int N = (int)G.size();
  T na = T(-1);

  vector< T > dist(N, na);
  priority_queue<P, vector<P>, greater<P> > pq;
  dist[start] = 0;
  pq.push({0, start});
  
  while (!pq.empty()) {
    auto [nowcost, nowv] = pq.top();
    pq.pop();

    if (dist[nowv] < nowcost) continue;

    for (auto &e : G[nowv]) {
      if (dist[e.to] == na || dist[nowv] + e.cost < dist[e.to]) {
            dist[e.to] = dist[nowv] + e.cost;
            pq.push({dist[e.to], e.to});
      }
    }
  }
  
  return dist;
}



