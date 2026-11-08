---
title: q二項係数・q階乗
documentation_of: //combinatorics/q_binomial.hpp
---

# q二項係数・q階乗（C-01）

[ヘッダ](../../combinatorics/q_binomial.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#q_binomial)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/q_binomial.hpp"
int main() {
    cp::QBinomial q(7, 2);
    std::cout << q(8, 3) << ' ' << q.factorial(8) << ' ' << q.zero_period() << '\n';
}
```

出力:

```text
2 0 3
```

## API・条件

`QBinomial(prime, q, max_prime=2000000)` は素数法と固定のqについて前計算します。qはsigned64から正規化します。`operator()(uint64 n,uint64 k)` はGaussian二項係数、`factorial(n)` は `[n]_q!`、`zero_period()` は `[d]_q=0` となる最初のd（存在しないq=0では0）。`modulus()` と `q()` は設定値です。k>nは0、空積は1。

素数以外は `invalid_argument`、pが表予算を超えると `length_error`。**小さい素数の表を再利用するAPI**であり、既定では998244353の全表は作りません。q=1の消失周期はpで、乗法位数1とは区別します。

## 仕組み

q=0なら全てのq整数は1。q=1なら通常のLucas二項係数です。それ以外はqの乗法位数dを求め、q-Lucas

`[n choose k]_q = C(floor(n/d),floor(k/d)) * [n%d choose k%d]_q`

を使います。0..d−1のq階乗だけを逆元表にするため、分母が0となる問い合わせでも0除算をしません。n≥dのq階乗そのものは0です。

## 計算量

前計算O(p)時間・O(p)空間（位数の計算を加算）。二項係数O(log_p(n+1))、階乗・設定参照O(1)。

## 使用する問題

有限体上の部分空間数を素数で割った余り、転倒数を重みにした多重集合順列、根付き単位根でのq多項式評価。

## 検証

q-Pascalの漸化式でp≤43の全qとn≤100を比較。q=0/1/−1、位数による0、uint64最大値、巨大表拒否。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
