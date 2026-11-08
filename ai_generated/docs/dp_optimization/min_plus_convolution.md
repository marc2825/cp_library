---
title: 凸 Min-Plus Convolution
documentation_of: //dp_optimization/min_plus_convolution.hpp
---

# 凸 Min-Plus Convolution

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#minplus) / [実装](../../dp_optimization/min_plus_convolution.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/min_plus_convolution.hpp"
int main() {
    auto c = cp::min_plus_convolution_arbitrary_convex(std::vector<long long>{3, 0, 4}, std::vector<long long>{0, 1, 4});
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/dp_optimization/min_plus_convolution.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`min_plus_convolution_convex_convex(a,b)` は両列が離散凸、`min_plus_convolution_arbitrary_convex(a,b)` はbだけ離散凸、`min_plus_convolution_convex_arbitrary(a,b)` はaだけ離散凸。結果c[k]=min(a[i]+b[j]), i+j=k、長さn+m−1。片方が空なら空。凸とは隣接差分が単調非減少で、条件の実行時検査はしません。有限値のみでINF番兵は提供しません。

標準整数の各候補の和の超過はoverflow_error、独自型はT(0)構築・コピー・+=・<が必要で算術は利用側管理。

## 計算量・原理

両凸はO(n+m)、片凸はO((n+m)log(n+m))時間、出力と再帰を含む追加O(n+m)領域。

両凸では2列の傾きを小さい順に取り込めます。片凸ではM[k,i]=a[i]+b[k−i]が有効帯内でMongeになり、最左最小位置が単調に移動します。無効三角領域を直接除いた分割統治でINFや危険な番兵加算を使わず処理します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
