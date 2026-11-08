---
title: Pell方程式
documentation_of: //number_theory_basic/pell_equation.hpp
---

# Pell方程式（N-23）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#pell_equation)

## API・条件

`pell_fundamental(uint64 D,sign=1,max_iterations=1000000)` はx²−Dy²=signの最小正整数解 `optional<PellSolution>{x,y}`。Dは正の非平方数、signは±1。−1側が解なしならnullopt、探索予算超過はlength_error。解はBigInt。`pell_solution_power(D,solution,uint64 exponent)` は(x+y√D)^exponent、指数0は(1,0)。normが±1でない入力はinvalid_argument。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/pell_equation.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/pell_equation.hpp"

int main() {
    const auto fundamental = cp::pell_fundamental(2);
    const auto third = cp::pell_solution_power(2, *fundamental, 3);
    std::cout << fundamental->x << ' ' << fundamental->y << '\n';
    std::cout << third.x << ' ' << third.y << '\n';
}
```

出力:

```text
3 2
99 70
```

## 原理・計算量

√Dの連分数を浮動小数点なしで進め、収束分数のnormを検査。周期長をLとするとO(L)任意精度演算、定数個のBigIntを保持します。Dだけから小さい反復回数を保証するAPIではありません。解の累乗はO(log exponent)回の環乗算。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

平方数条件の整数列、大きな最小解を持つ二次不定方程式。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
