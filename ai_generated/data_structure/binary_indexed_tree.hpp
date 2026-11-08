// Legacy BIT: public n/bit/val are retained. Prefer range_query/fenwick_tree.hpp.
// Docs and migration: docs/range_query/fenwick_tree.md
#pragma once
#include <vector>
#include <cassert>

using namespace std;


/// Binary Indexed Tree
// 0-indexed / 半開区間 で外からはアクセスできるようにしている（内部では1-indexedで取り扱っている）
template <class T>
struct BIT {
    int n;          // 配列の要素数（1-indexedのため、数列の要素数+1する）
    vector<T> bit;  // BITデータ(1-indexed)の格納先
    vector<T> val;  // 実データ(1-indexed)の格納先
    BIT(int n_) : n(n_ + 1), bit(n, 0), val(n, 0) {} // 1-indexed

    // A_i += x (0-indexed)
    void add(int idx, T x) {
        assert(0 <= idx && idx < n - 1);
        idx++;
        val[idx] += x;
        while(idx < n){
            bit[idx] += x;
            idx += (idx & -idx);
        }
    }

    // get A_i (0-indexed)
    T get(int idx) {
        assert(0 <= idx && idx < n - 1);
        return val[idx+1];
    }

    // A_i = x (0-indexed)
    void set(int idx, T x) {
        add(idx, x-get(idx));
    }

    // A_0 ~ A_(idx-1) の和を計算 (idx含まない) (0-indexed / 半開区間)
    T sum(int idx) {
        assert(0 <= idx && idx < n);
        T ret = 0;
        while(idx > 0){ 
            ret += bit[idx];
            idx -= (idx & -idx);
        }
        return ret;
    }

    // A[l,r)の和を計算 (0-indexed / 半開区間)
    T sum(int l, int r) {
        return sum(r) - sum(l);
    }

    // A_0 + A_1 + ... + A_x >= w となるような最小の x を求める (A_i >= 0) (0-indexedで返している)
    int lower_bound(T w) { 
    if (w <= 0) return 0;
    else {
        int x = 0, r = 1; // xでギリギリNG
        while (r < n) r = r << 1;
        for (int len = r; len > 0; len = len >> 1) { 
            if (x + len < n && bit[x + len] < w) { 
                w -= bit[x + len];
                x += len;
                }
            }

            return x; // 0-indexedで返す
        }
    }
};
