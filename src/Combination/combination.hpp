#pragma once
#include <vector>
#include <cassert>

namespace mylib {

/**
 * @brief 二項係数 (nCk mod p)
 * @docs docs/combination.md
 * @note 前計算 O(N), クエリ O(1)。逆元テーブルを線形時間で構築する。
 */
struct Combination {
    int MAX;
    int MOD;
    std::vector<long long> fac, finv, inv;

    /**
     * @brief コンストラクタ
     * @param limit 計算するnの最大値 (MAX)
     * @param mod 法とする素数
     */
    Combination(int limit, int mod) : MAX(limit), MOD(mod) {
        fac.resize(MAX + 1);
        finv.resize(MAX + 1);
        inv.resize(MAX + 1);
        init();
    }

    void init() {
        fac[0] = fac[1] = 1;
        finv[0] = finv[1] = 1;
        inv[1] = 1;
        for (int i = 2; i <= MAX; i++) {
            fac[i] = fac[i - 1] * i % MOD;
            inv[i] = MOD - inv[MOD % i] * (MOD / i) % MOD;
            finv[i] = finv[i - 1] * inv[i] % MOD;
        }
    }

    /**
     * @brief nCk (組み合わせ) を計算する
     */
    long long nCk(int n, int k) const {
        if (n < k || n < 0 || k < 0) return 0;
        return fac[n] * (finv[k] * finv[n - k] % MOD) % MOD;
    }

    /**
     * @brief nPk (順列) を計算する
     */
    long long nPk(int n, int k) const {
        if (n < k || n < 0 || k < 0) return 0;
        return fac[n] * finv[n - k] % MOD;
    }

    /**
     * @brief nHk (重複組み合わせ) を計算する
     */
    long long nHk(int n, int k) const {
        if (n == 0 && k == 0) return 1;
        return nCk(n + k - 1, k);
    }
};

} // namespace mylib