/*
★ Name         : 座標圧縮
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description : （3つ以下の）配列に対して、in-placeに座標圧縮(0-indexed)する
                  返り値は座標圧縮後の個数（最大値-1）
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc036/submissions/54997951
 ■ References  :

 ■ TODO        : 省メモリにするためにmapではなくunique使う方針でやってもいいかも
*/


#pragma once
#include <vector>
#include <map>

using namespace std;

/// 1つの配列に対して、in-placeに座標圧縮(0-indexed)する
// 返り値は座標圧縮後の個数（最大値-1）
template<class T>
int zaatu(vector<T> &A) {
    map<T,int> mp;
    for(auto &x: A) mp[x] = 1;

    int cnt = 0;
    for(auto &[x,v]: mp) v = cnt++;

    for(auto &x: A) x = mp[x];
    return cnt;
}

/// 2つの配列に対して、in-placeに座標圧縮(0-indexed)する
// 返り値は座標圧縮後の個数（最大値-1）
template<class T>
int zaatu(vector<T> &A, vector<T> &B) {
    map<T,int> mp;
    for(auto &x: A) mp[x] = 1;
    for(auto &x: B) mp[x] = 1;

    int cnt = 0;
    for(auto &[x,v]: mp) v = cnt++;

    for(auto &x: A) x = mp[x];
    for(auto &x: B) x = mp[x];
    return cnt;
}

/// 3つの配列に対して、in-placeに座標圧縮(0-indexed)する
// 返り値は座標圧縮後の個数（最大値-1）
template<class T>
int zaatu(vector<T> &A, vector<T> &B, vector<T> &C) {
    map<T,int> mp;
    for(auto &x: A) mp[x] = 1;
    for(auto &x: B) mp[x] = 1;
    for(auto &x: C) mp[x] = 1;

    int cnt = 0;
    for(auto &[x,v]: mp) v = cnt++;

    for(auto &x: A) x = mp[x];
    for(auto &x: B) x = mp[x];
    for(auto &x: C) x = mp[x];
    return cnt;
}