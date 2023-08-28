/*
★ Name         : Binary Indexed Tree
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


/// Binary Indexed Tree
template <typename T>
struct BIT {
    int n;          // 配列の要素数(数列の要素数+1)
    vector<T> bit;  // データの格納先
    BIT(int n_) : n(n_ + 1), bit(n, 0) {} // 1-indexed

    // A_i += x
    void add(int idx, T x) {
        while(idx < n){ // n <- n+1 に予めしてるため等号を含まないことに注意
            bit[idx] += x;
            idx += (idx & -idx);
        }
    }

    // A_1 ~ A_i の和を計算
    T sum(int idx) {
        T ret(0);
        while(idx > 0){ 
            ret += bit[idx];
            idx -= (idx & -idx);
        }
        return ret;
    }

    // A_1 + A_2 + ... + A_x >= w となるような最小の x を求める (A_i >= 0)
    int lower_bound(T w) { 
    if (w <= 0) return 0;
    else {
        int x = 0, r = 1;
        while (r < n) r = r << 1;
        for (int len = r; len > 0; len = len >> 1) { 
            if (x + len < n && bit[x + len] < w) { 
                w -= bit[x + len];
                x += len;
                }
            }
            return x + 1;
        }
    }
};