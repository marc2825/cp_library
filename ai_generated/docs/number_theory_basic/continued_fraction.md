---
title: 連分数・有理数近似
documentation_of: //number_theory_basic/continued_fraction.hpp
---

# 連分数・有理数近似（N-22）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#continued_fraction)

## API・条件

`continued_fraction(BigRational)` は正則連分数、先頭のみ負可、末尾1を使わない標準形。`continued_fraction_convergents(vector<BigInt>)` は全収束分数、空入力なら空です。2項目以降は正整数。`best_rational_approximation(value,bound)` は分母≤boundで絶対誤差最小の有理数。同点では分母が小さいもの、さらに同点なら値が小さいものを選びます。bound≥1。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/continued_fraction.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/continued_fraction.hpp"

int main() {
    const cp::BigRational value(355, 113);
    for (const auto& a : cp::continued_fraction(value)) std::cout << a << ' ';
    std::cout << '\n' << cp::best_rational_approximation(value, 100) << '\n';
}
```

出力:

```text
3 7 16
311/99
```

## 原理・計算量

Euclid商列とp_i=a_i p_{i−1}+p_{i−2}, q_i=a_i q_{i−1}+q_{i−2}。分母制限を超える直前の収束分数と半収束分数を厳密に比較します。分母dに対しO(log d)回のBigInt演算。展開/収束列はO(log d)個の値、近似だけなら定数個を保持。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

有理数の簡約、誤差最小の小分母近似、連分数に基づく整数方程式。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
