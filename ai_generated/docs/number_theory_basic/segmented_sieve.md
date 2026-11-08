---
title: 区間篩
documentation_of: //number_theory_basic/segmented_sieve.hpp
---

# 区間篩（N-01）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#segmented_sieve)

## API・条件

`SegmentedSieve(left,right,max_width=1000000,max_root=5000000)` はuint64の半開区間を保持します。`primes()` は昇順素数列、`factorization(x)` は昇順の `(prime,exponent)` 列をconst参照で返します。空区間可。0の分解はdomain_error、1は空、範囲外はout_of_range。右端UINT64_MAXは指定できますが、その値自身を区間に含めることはできません。幅と√(right−1)の予算超過はlength_error。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/segmented_sieve.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/segmented_sieve.hpp"

int main() {
    cp::SegmentedSieve sieve(100, 111);
    for (auto p : sieve.primes()) std::cout << p << ' ';
    std::cout << '\n';
    for (auto [p, e] : sieve.factorization(108)) std::cout << p << ' ' << e << ' ';
    std::cout << '\n';
}
```

出力:

```text
101 103 107 109
2 2 3 3
```

## 原理・計算量

R=right、W=right−left、s=⌊√(R−1)⌋。sまでの篩を作り、各素数の最初の倍数から割り切れるだけ除去します。前処理はO(s log log s + π(s) + W log R)、領域O(s+W+K)、Kは保存する素因数組の総数。空区間はO(1)。分解取得はO(1)、返却参照はオブジェクトの寿命まで有効です。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

大きな整数付近の連続区間に対する素数列挙、約数数・Euler φの一括計算。単一整数には既存Pollard–Rhoを使います。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
