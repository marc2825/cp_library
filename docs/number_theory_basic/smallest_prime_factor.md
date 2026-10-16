---
title: SPF・最小素因数
documentation_of: //number_theory_basic/smallest_prime_factor.hpp
---

# SPF・最小素因数

線形篩で各整数の最小素因数を作ります。整数iと `p<=spf[i]` の素数だけを組み合わせ、合成数i*pへ最小素因数pを一度だけ書き込みます。前処理後は分解を繰り返し高速に行えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#spf) / [実装](../../number_theory_basic/smallest_prime_factor.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/smallest_prime_factor.hpp"
int main() {
    cp::SmallestPrimeFactor spf(100);
    std::cout << spf[84] << '\n';
    for (auto [prime, exponent] : spf.factorize(84))
        std::cout << prime << '^' << exponent << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/smallest_prime_factor.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`SmallestPrimeFactor(int limit=0)`

| メンバ | 内容 |
|---|---|
| `limit()` | 上限N |
| `operator[](int n)` | 最小素因数。0,1の値はそれぞれ0,1 |
| `is_prime(int n)` | n>=2かつspf[n]==n |
| `table()`, `primes()` | const参照の `vector<int>`、表と昇順素数列 |
| `factorize(int n)` | 昇順の `vector<pair<long long,int>>`、(素数,指数) |

- `limit>=0`、負の上限は `invalid_argument`。0と1も構築できます。
- 表・素数判定は `[0,N]`、範囲外は `out_of_range`。
- 分解は `1<=n<=N`。n<=0は `invalid_argument`、上限超過は `out_of_range`。1の分解は空です。
- 試し割りの `prime_factorization` と同じ戻り値の型です。上限より大きい値を残った素数とみなす等の推測は行いません。

構築O(N)時間・O(N)語の空間、表の参照・判定O(1)、分解O(log(n+1))時間・出力サイズ分の空間。各除算でnが少なくとも半分になります。普通の篩より多くのメモリを使います。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/abc179/tasks/abc179_c)：AtCoder ABC179 C — A x B + C。各ABの約数個数をSPFで求めます。

0〜10,000のSPFを最初の約数の走査と比較し、全分解と素数列、小さい上限・範囲外・200万の表を確認します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
