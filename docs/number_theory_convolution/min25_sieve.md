---
title: 一般化Min_25 Sieve
documentation_of: //number_theory_convolution/min25_sieve.hpp
---

# 一般化Min_25 Sieve（N-26）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#min25_sieve)

## API・条件

`min25_sieve<T>(N,prime_coefficients,prime_power,max_root=5000000,max_degree=12)` は全商のΣ_{i≤x}f(i)。fは乗法的、f(1)=1、f(p)=Σc_j p^j、callback `(uint64 p,unsigned e,uint64 power)->T` はf(p^e)。両方の指定は一致する必要があります。`polynomial_prime_sum_table<T>` は素数項の表だけを返します。空の係数列はf(p)=0。TはBigInt・組込み整数・modint等の可換環。modintは合成数法も可。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_convolution/min25_sieve.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_convolution/min25_sieve.hpp"

int main() {
    const auto totient = cp::min25_sieve<cp::BigInt>(100, {-1, 1},
        [](auto p, unsigned, auto power) { return cp::BigInt(power / p * (p - 1)); });
    std::cout << totient.total() << '\n';
}
```

出力:

```text
3044
```

## 原理・計算量

各p^jの素数和をLucy篩で作り線形結合、素数を逆順に処理して最小素因数の冪を復元します。次数d一定なら保守的にO((d+1)N^(3/4))環演算、領域O(√N+d²)。整数冪和はStirling数と二項係数で厳密に計算して環へ写すため、合成数法の除算は不要。BigIntでの初期冪和計算コストO(d²√N)を別途含みます。整数Tへの変換超過はout_of_range、環演算超過はoverflow_error。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

φ、約数個数、約数和、素数冪の値だけから指定する大規模総和。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
