---
title: SMAWK
documentation_of: //dp_optimization/smawk.hpp
---

# SMAWK

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#smawk) / [実装](../../dp_optimization/smawk.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/smawk.hpp"
int main() {
    auto a = cp::smawk(4, 6, [](int r, int c) { return (r - c) * (r - c); });
    for (int c : a) std::cout << c << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/dp_optimization/smawk.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`smawk(rows,cols,get)->vector<int>`。**全単調**な行列の最左行最小位置を返します。全ての部分行列で最左argminが単調非減少であることが必要で、行列全体のargmin単調性だけでは不十分。Monge行列なら可。getは任意順に呼出されても値が変わらないこと。最左同点、0行／0列／負次元の仕様はMonotone Minimaと同じ。

## 計算量・原理

O(rows+cols)回の値評価・比較、O(rows+cols)補助領域。getの費用は別。

候補列を行数以下へreduceし、奇数位置の行を再帰的に求め、偶数位置の行を隣の解の間だけ走査します。同点で古い列を消さず最左解を保ちます。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
