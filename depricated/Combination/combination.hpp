/*
★ Name         : 二項係数の計算　(Combination nCk % mod)
 ■ Snipet      : Yes
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  :

 ■ TODO        : MODがないバージョン（普通に計算する？）を作っておく

*/


#pragma once
#include <vector>

using namespace std;

/// 二項係数
struct Combination {
    int MAX;
    int MOD;
    vector<long long> fac,finv,inv;

    Combination(int MAX, int MOD) : MAX(MAX + 1), MOD(MOD) {
        fac.resize(MAX + 1);
        finv.resize(MAX + 1);
        inv.resize(MAX + 1);
        COMinit();
    }

    void COMinit() {
        fac[0] = fac[1] = 1;
        finv[0] = finv[1] = 1;
        inv[1] = 1;
        for (int i = 2; i < MAX; i++) {
            fac[i] = fac[i - 1] * i % MOD;
            inv[i] = MOD - inv[MOD%i] * (MOD / i) % MOD;
            finv[i] = finv[i - 1] * inv[i] % MOD;
        }
    }

    // 二項係数計算
    long long COM(int n, int k) {
        if (n < k) return 0;
        if (n < 0 || k < 0) return 0;
        return fac[n] * (finv[k] * finv[n - k] % MOD) % MOD;
    }
};