---
title: Batch Inversion
documentation_of: //number_theory_basic/batch_inversion.hpp
---

# Batch Inversion（N-05）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#batch_inversion)

## API・条件

`batch_inverse<T>(values,inverse)` は可換環Tで、callback `inverse(product)->optional<T>` を1回呼びます。全要素の逆元列、または積が非unitならnullopt。空入力は空でcallbackを呼びません。`batch_inverse_mod(values,m)` はuint64全域の正の法、合成数・法1を含みます。0は法1だけで可逆。m=0はinvalid_argument。汎用版Tの通常積が正しく表現できることが必要です。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/batch_inversion.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/batch_inversion.hpp"

int main() {
    const auto inverses = cp::batch_inverse_mod({3, 5, 7}, 16);
    for (auto x : *inverses) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
11 13 7
```

## 原理・計算量

prefix積と積全体の逆元から逆順に各逆元を復元。O(n)回の積と1回の逆元、領域O(n)。mod版は共通のoverflowを避ける剰余乗算とBigInt拡張Euclidを共有します。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

補間、複数分母の除算、合成数法でunitだけを扱う計算。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
