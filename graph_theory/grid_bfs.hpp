/*
★ Name         : Grid BFS
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc317/submissions/44957185
 ■ References  :

 ■ TODO        :
*/


#pragma once
#include <vector>
#include <queue>
#include <string>

using namespace std;


#define include(y, x, H, W) (0 <= (y) && (y) < (H) && 0 <= (x) && (x) < (W))
const vector<int> dx1 = {1,0,-1,0};
const vector<int> dy1 = {0,1,0,-1};

// Returns the distance of all points on the graph from the starting point
// 二次元配列での (i,j) が (y,x) と対応 (二次元座標と同じ)
// 上下左右4方向への探索、通行不可文字はngで指定（適宜書き換えても良い）
vector<vector<int> > Grid_BFS(const vector<string> &G, int sy, int sx, char ng = '#') {
    int H = G.size();
    int W = G[0].size();
    vector<vector<int> > dist(H, vector<int>(W, -1));
    dist[sy][sx] = 0;
    queue<pair<int, int> > q;
    q.push({sy,sx});

    while(!q.empty()) {
        auto [y,x] = q.front();
        q.pop();

        for(int k = 0; k < 4; k++) {
            int ny = y + dy1[k];
            int nx = x + dx1[k];

            if(include(ny,nx,H,W) && dist[ny][nx] == -1 && G[ny][nx] != ng) {
                dist[ny][nx] = dist[y][x] + 1;
                q.push({ny,nx});
            }
        }
    }

    return dist;
}