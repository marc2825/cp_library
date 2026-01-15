/*
★ Name         : Union-Find (Disjoint Set Union)
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  :

 ■ TODO        :
*/

#pragma once
#include <vector>

using namespace std;


// Union-Find
struct unionfind{
    vector<int> par, siz;
 
    // 初期化
    unionfind(int n) : par(n, -1), siz(n, 1) {}
 
    // 根を求める
    int root(int x) {
        if (par[x] == -1) return x;
        else return par[x] = root(par[x]);
    }
 
    // xとyの根（グループ）が一致するかどうか
    bool issame(int x, int y){
        return root(x) == root(y);
    }
 
    // xとyのグループの併合
    bool unite(int x, int y){
        x = root(x); y = root(y);
 
        if (x == y) return false;
 
        if (siz[x] < siz[y]) swap(x,y);
 
        par[y] = x;
        siz[x] += siz[y];
        return true;
    }
 
    // xを含むグループのサイズ
    int size(int x){
        return siz[root(x)];
    }
};