/*
★ Name        ： 単一始点最短路問題ソルバー（Dijkstra法（復元付き）by priority_queue、Bellman-Ford法、Floyd–Warshall法）
 ■ Snipet      : No

 ■ Arguments   : 
 ■ Return      : 
 ■ Complexity  : 

 ■ Description : 
 

 ■ Usage       : 


 ■ Verify      : https://atcoder.jp/contests/abc035/submissions/44597953
 ■ References  : https://ei1333.github.io/library/graph/shortest-path/dijkstra.hpp

 ■ TODO (Main) : graph_template とどう共存させるか（あっちは部分問題での超典型処理を瞬殺するためのもので柔軟性は微妙なので、スニペット的でいいかも）
*/


#pragma once
#include <vector>
#include <queue>
#include <limits>
#include "graph_template.hpp"

using namespace std;


template< typename T >
struct ShortestPath {
  vector< T > dist;
  vector< int > from, id;
};

// 0-indexed
// Returns the minimum distance of all points on the graph from the starting point 
template <typename T>
ShortestPath< T > dijkstra(const Graph< T > &g, int start = 0) {
  using P = pair<T, int>;
  int N = (int)g.size();

  vector<T> dist(N, T(-1));
  priority_queue<P, vector<P>, greater<P> > pq;
  dist[start] = 0;
  pq.push({0, start});
  
  while (!pq.empty()) {
    auto [nowcost, nowv] = pq.top();
    pq.pop();

    if (dist[nowv] < nowcost) continue;

    for (auto &e : g[nowv]) {
      if (dist[e.to] == T(-1) || dist[nowv] + e.cost < dist[e.to]) {
            dist[e.to] = dist[nowv] + e.cost;
            pq.push({dist[e.to], e.to});
      }
    }
  }
  
  return dist;
}



