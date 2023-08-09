/*
★ Name        ： Dijkstra法　 基本形 by priority_queue
 ■ Snipet      : Yes

 ■ Arguments   : Graph(V,E) (vector<vector<int> >), start point (int)
 ■ Return      : distances from start (vector<int>)
 ■ Complexity  : 

 ■ Explanation : 
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc309/submissions/44418398
 ■ References  :
*/


#pragma once
#include <vector>
#include <queue>

using namespace std;

struct Edge{
    int to;
    long long cost;

    Edge(int to = 0, long long cost = 0):to(to), cost(cost){}

};

using Graph = vector<vector<Edge> >;


// 0-indexed
// Returns the distance of all points on the graph from the starting point 
template <typename T>
vector<T> dijkstra(Graph &g, int start = 0) {
  using P = pair<T, int>;
  int N = (int)g.size();
  vector<T> d(N, T(-1));
  priority_queue<P, vector<P>, greater<P> > Q;
  d[start] = 0;
  Q.emplace(0, start);
  while (!Q.empty()) {
    auto [nowcost, nowv] = Q.top();
    Q.pop();

    if (d[nowv] < p.first) continue;
    for (auto e : g[now]) {
      if (d[e.to] == T(-1) || d[nowv] + e.cost < d[e.to]) {
        d[e.to] = d[nowv] + dst.cost;
        Q.emplace(d[e.to], e.to);
      }
    }
  }
  
  return d;
}