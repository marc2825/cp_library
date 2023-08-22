/*
★ Name        ：行列積
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