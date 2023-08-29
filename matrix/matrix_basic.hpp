/*
★ Name        ：行列に関する基本関数（積、累乗、回転）
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
#include <cassert>

using namespace std;


/// 行列A・B同士の積 (modが必要ならば指定する、不要ならそのまま)
template<typename T>
vector<vector<T> > matrix_prod(const vector<vector<T> > &A, const vector<vector<T> > &B, const int mod = -1){
    // Aは i * k , Bは k * j の行列 -> 積で i * j の行列を返す
    assert(A[0].size() == B.size());
    int r = A.size();
    int c = B[0].size();

    vector<vector<T> > ret(r, vector<T>(c,0));
    for(int i = 0; i < r; i++){
        for(int j = 0; j < c; j++){
            for(int k = 0; k < (int)B.size(); k++){
                ret[i][j] += A[i][k] * B[k][j];
                if(mod > 0) ret[i][j] %= mod;
            }
        }
    }
    return ret;
}

/// 行列累乗 (A^k)
template <typename T> 
void matrirx_pow(vector<vector<T> > A, long long k) {
    if(A.empty()) return A;
    vector<vector<T> > ret(A.size(), vector<T>(A[0].size()));
    for(int i = 0; i < A.size(); i++) ret[i][i] = 1;

    while(k > 0) {
        if(k & 1) ret = matrix_prod(ret, A);
        A = matrix_prod(A, A);
        k >>= 1;
    }
    return ret;
}

/// 反時計周りに 90 度回転
template <typename T> 
void matrix_rotate_anticlock(vector<vector<T> > &A) {
    if(A.empty() || A[0].empty()) return;
    int R = A.size();
    int C = A[0].size();
    vector<vector<T> > ret(C, vector<T>(R));
    for(int i = 0; i < n; i++) 
        for(int j = 0; j < m; j++) ret[m - 1 - j][i] = A[i][j];
    swap(A, ret);
}


// 行列を時計回りに90度回転
template<typename T>
vector<vector<T> > matrix_rotate_clock(vector<vector<T> > &A) {
    if(A.empty() || A[0].empty()) return;
    int R = A.size();
    int C = A[0].size(); 
    vector<vector<T> > ret(C, vector<T>(R));
    for(int i = 0; i < R; i++)
        for(int j = 0; j < C; j++) ret[j][R-1-i] = A[i][j];
    swap(A, ret);
}