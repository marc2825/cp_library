/*
★ Name        ： Breadth First Search (幅優先探索)　 基本形
 ■ Snipet      : Yes

 ■ Arguments   : Graph(V,E) (vector<vector<int> >), start point (int)
 ■ Return      : distances from start (vector<int>)
 ■ Complexity  : 

 ■ Description : 
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc309/submissions/44418398
 ■ References  :

 ■ TODO (Main) :
*/


#pragma once
#include <vector>
#include <queue>

using namespace std;


// 0-indexed
// Returns the distance of all points on the graph from the starting point 
vector<int> BFS(const vector<vector<int> > &G, int start) {
    vector<int> dist(G.size(), -1);
    dist[start] = 0;
    queue<int> q;
    q.push(start);

    while(!q.empty()) {
        int now = q.front();
        q.pop();

        for(auto &nxt : G[now]) {
            if(dist[nxt] == -1) {
                dist[nxt] = dist[now] + 1;
                q.push(nxt);
            }
        }
    }

    return dist;
}
