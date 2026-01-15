/*
★ Name         : modが素数でない時の 二項係数の計算　(Combination nCk % mod)
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc284/submissions/45318168
 ■ References  : https://qiita.com/suisen_cp/items/d0ab7e728b98bbec818f

 ■ TODO        : https://nyaannyaan.github.io/library/modulo/arbitrary-mod-binomial.hpp.html を参考にして改良する？
*/


#pragma once
#include <vector>
#include <numeric>


using namespace std;


/// https://qiita.com/suisen_cp/items/d0ab7e728b98bbec818f

// referece: https://37zigen.com/linear-sieve/
class LinearSieve {
    public:
        LinearSieve(unsigned int n) : _n(n), min_prime_factor(std::vector<unsigned int>(n + 1)) {
            std::iota(min_prime_factor.begin(), min_prime_factor.end(), 0);
            prime_list.reserve(_n + 1);
            for (unsigned int d = 2; d <= _n; ++d) {
                if (min_prime_factor[d] == d) {
                    prime_list.push_back(d);
                }
                unsigned int prime_max = std::min(min_prime_factor[d], _n / d);
                for (unsigned int prime : prime_list) {
                    if (prime > prime_max) {
                        break;
                    }
                    min_prime_factor[prime * d] = prime;
                }
            }
        }
        unsigned int prime_num() const {
            return prime_list.size();
        }
        const std::vector<unsigned int>& get_prime_list() const {
            return prime_list;
        }
        const std::vector<unsigned int>& get_min_prime_factor() const {
            return min_prime_factor;
        }
    private:
        const unsigned int _n;
        std::vector<unsigned int> min_prime_factor;
        std::vector<unsigned int> prime_list;
};

// nCk mod m を全ての k = 0, ..., n に対して求める．m は素数でなくてもよい．
class ArbitraryModBinomialCoefficients {
    public:
        // nCk mod m を求める
        ArbitraryModBinomialCoefficients(unsigned int N, unsigned int M) :
            _N(N), _M(M), _sieve(LinearSieve(N)), _binom(std::vector<long long>(N + 1)) {
            solve();
        }
        // return nCk
        long long operator[](unsigned int k) const {
            return _binom[k];
        }
        // return nC0, nC1, ..., nCn mod m を格納した vector
        const std::vector<long long>& get_coeffs() const {
            return _binom;
        }
        // 副産物としての線形篩
        const LinearSieve& get_sieve() const {
            return _sieve;
        }
    private:
        const unsigned int _N, _M;
        const LinearSieve _sieve;
        std::vector<long long> _binom;

        // return y = (x mod M) (0 <= y < M)
        inline unsigned int safe_mod(const long long x) {
            int y = x % _M;
            return y < 0 ? y + _M : y;
        }

        // return x s.t. 0 <= x < m and (v * x mod m) = gcd(a, m)
        unsigned int gcd_inv(const unsigned int v) {
            long long a = v, b = _M;
            long long x = 1, z = 0;
            long long y = 0, w = 1;
            long long tmp;
            while (b) {
                long long p = a / b, q = a % b;
                tmp = x - z * p; x = z; z = tmp;
                tmp = y - w * p; y = w; w = tmp;
                a = b; b = q;
            }
            return safe_mod(x);
        }

        // reference: https://37zigen.com/linear-sieve/
        // 合成数 mod に対して逆元のテーブルを求める
        // 逆元が存在しない添え字に対して，値は未定義
        void mod_invs(std::vector<long long>& invs) {
            auto &mpf = _sieve.get_min_prime_factor();
            for (unsigned int i = 0; i <= _N; ++i) {
                unsigned int pf = mpf[i];
                if (pf == i) {
                    invs[i] = gcd_inv(i);
                } else {
                    invs[i] = invs[pf] * invs[i / pf] % _M;
                }
            }
        }

        void solve() {
            // sieve は線形篩
            // 最小素因数のテーブルと素数のリストを持つ
            auto &primes = _sieve.get_prime_list();
            // d と p を求める
            std::vector<unsigned int> d(_N + 1, 0);
            std::vector<unsigned int> p;
            for (unsigned int prime : primes) {
                if (_M % prime) continue;
                p.push_back(prime);
                unsigned int sz = p.size();
                for (unsigned int v = prime; v <= _N; v += prime) {
                    d[v] = sz;
                }
            }
            const unsigned int L = p.size();
            // p は 1-indexed なのでダミー要素を先頭に入れる
            p.insert(p.begin(), 0);
            // invs[k] = k ^ {-1} mod M (存在する場合)
            // 存在しない場合の値は未定義だが，アクセスすることはない．
            std::vector<long long> invs(_N + 1);
            mod_invs(invs);
            // べき乗の前計算
            std::vector<std::vector<long long>> powers(L + 1);
            for (unsigned int i = 1; i <= L; ++i) {
                // 指数 (べき乗の肩の値) の上界
                unsigned int max_index = _N / (p[i] - 1);
                powers[i].resize(max_index + 1);
                powers[i][0] = 1;
                for (unsigned int j = 0; j < max_index; ++j) {
                    powers[i][j + 1] = powers[i][j] * p[i] % _M;
                }
            }
            // 定数倍高速化のため，前半分だけを漸化式に従って計算する
            const unsigned int half = (_N + 1) / 2;
            // k = 0 の場合について，S, T を初期化
            long long S = 1;
            std::vector<unsigned int> T(L + 1, 0);
            // nC0 = 1
            _binom[0] = 1;
            for (unsigned int k = 1; k <= half; ++k) {
                // num: 漸化式の分子, den: 漸化式の分母
                unsigned int num = _N - k + 1, den = k;
                // T の更新
                // t を求めず，直接 T に足しこむことで余分な領域を使わずに済ませる
                while (d[num]) ++T[d[num]], num /= p[d[num]];
                while (d[den]) --T[d[den]], den /= p[d[den]];
                // S の更新
                S = S * num % _M;
                S = S * invs[den] % _M;
                // S, T から nCk を復元
                _binom[k] = S;
                for (unsigned int i = 1; i <= L; ++i) {
                    _binom[k] = _binom[k] * powers[i][T[i]] % _M;
                }
            }
            // 後ろ半分は nCk = nCn-k を用いて求めると定数倍がいい
            for (unsigned int k = half + 1; k <= _N; ++k) {
                _binom[k] = _binom[_N - k];
            }
        }
};