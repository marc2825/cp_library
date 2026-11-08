/*
★ Name        ：二次元累積和
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/tessoku-book/submissions/54725528
                 https://atcoder.jp/contests/tessoku-book/submissions/54725535
                 https://atcoder.jp/contests/arc025/submissions/54725557
 ■ References  : https://ei1333.github.io/library/dp/cumulative-sum-2d.hpp.html

 ■ TODO        :
*/


#pragma once
#include <vector>

using namespace std;


/// 二次元累積和
// 0-indexed / 半開区間 で外からはアクセス、内部では1-indexed的に累積和を処理
template< class T >
struct CumulativeSum2D {
    vector< vector< T > > data;

    CumulativeSum2D(int H, int W) : data(H + 1, vector< T >(W + 1, 0)) {}

    // 0-indexedでアクセス
    void add(int x, int y, T z) {
        x++, y++;
        if(x >= (int)data.size() || y >= (int)data[0].size()) return;
        data[x][y] += z;
    }

    void build() {
        for(int i = 1; i < (int)data.size(); i++) {
            for(int j = 1; j < (int)data[i].size(); j++) {
                data[i][j] += data[i][j - 1] + data[i - 1][j] - data[i - 1][j - 1];
            }
        }
    }

    // 端を (sx, sy) ~ (gx, gy) とするような矩形領域の和を求める
    // 0-indexedでアクセス、半開区間扱いなので、gx/gyの行/列の値は含まれないことに注意
    T query(int sx, int sy, int gx, int gy) const {
        return (data[gx][gy] - data[sx][gy] - data[gx][sy] + data[sx][sy]);
    }
};