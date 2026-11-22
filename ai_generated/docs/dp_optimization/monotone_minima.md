---
title: Monotone Minima
documentation_of: //dp_optimization/monotone_minima.hpp
---

# Monotone Minima

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#monotone) / [実装](../../dp_optimization/monotone_minima.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/monotone_minima.hpp"
int main() {
    auto a = cp::monotone_minima(4, 6, [](int r, int c) { return (r - c) * (r - c); });
    for (int c : a) std::cout << c << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/dp_optimization/monotone_minima.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`monotone_minima(rows,cols,get)->vector<int>`。get(row,col)は変更しない行列値、値の `<` だけを使います。**各行の最左argminが単調非減少**であることが必要。Mongeは十分条件。最左の同点を選択。0行は空、0列では全行−1、負次元invalid_argument。条件の実行時検査はしません。

## 計算量・原理

O(rows+cols log(rows+1))回の値評価、O(rows+log(rows+1))追加領域。getの費用を掛けます。

中央行を走査し、そのargminを上下の行の探索境界にします。DPの各段で前段が確定している遷移行列に使えます。getがまだ計算されていない同じ段のDP値を読むような呼出しは不可です。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary)。帯状 Monge 行列 `a[i]+b[k−i]` の行最小として解けますが、帯の外の扱いを含む完成版は[min_plus_convolution_arbitrary_convex](min_plus_convolution.md)です。

[追加分の検証入口](../../verify/run_advanced.py)で、`(r−c)²+offsets[c]+row[r]` 型の Monge 行列700個（70×60未満）を素朴な最左 argmin と比較します（`verify/advanced/algebra_test.cpp`）。満たさない行列でも範囲内の列を返しますが、最小とは限りません。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
