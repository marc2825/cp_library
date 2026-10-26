---
title: 再利用可能な離散対数表
documentation_of: //number_theory_basic/discrete_log_table.hpp
---

# 再利用可能な離散対数表（N-08）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#discrete_log_table)

## API・条件

`DiscreteLogTable(base,modulus,max_baby_steps=2000000)` は底と法を両方固定します。uint64全域、正の法。`query(target)` は最小x≥0で base^x≡target、なければnullopt。非互いに素の底にも対応。法1の答えは0、0^0=1。底が変わる場合は別の表を構築します。`baby_steps()` は保存した異なる値の数です。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/discrete_log_table.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/discrete_log_table.hpp"

int main() {
    cp::DiscreteLogTable logarithm(2, 12);
    for (auto b : {1, 2, 4, 8, 3}) {
        const auto x = logarithm.query(b);
        std::cout << (x ? static_cast<long long>(*x) : -1) << ' ';
    }
    std::cout << '\n';
}
```

出力:

```text
0 1 2 3 -1
```

## 原理・計算量

gcdを剥がす段階列と、縮小した法m′のbaby tableを一度構築します。前処理・領域はO(√m′+log m)、各queryは期待O(√m′+log m)のハッシュ/整数演算。最小のbaby添字を保持して最小解を返します。√m′の予算を超えるとlength_error。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

同じ乗法列に対する多数の到達時刻問い合わせ。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
