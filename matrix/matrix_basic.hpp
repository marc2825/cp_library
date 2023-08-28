/*
★ Name        ：行列に関する基本関数（積、累乗、回転）
 ■ Snipet      : 
 
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
#include <cassert>

using namespace std;


/// 行列同士の積
template<typename T>
vector<vector<T> > matrix_prod(vector<vector<T> > &A, vector<vector<T> > &B){
    // Aは i * k , Bは k * j の行列 -> 積で i * j の行列を返す
    assert(A[0].size() == B.size());
    int r = A.size();
    int c = B[0].size();

    vector<vector<T> > ret(r, vector<T>(c,0));
    rep(i,r){
        rep(j,c){
            rep(k,B.size()){
                ret[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return ret;
}

/// 反時計周りに 90 度回転
template <typename T> 
void rot(vector<vector<T>> &v) {
    if(empty(v)) return;
    int n = v.size(), m = v[0].size();
    vector<vector<T>> res(m, vector<T>(n));
    rep(i, n) rep(j, m) res[m - 1 - j][i] = v[i][j];
    v.swap(res);
}