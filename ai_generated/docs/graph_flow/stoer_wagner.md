---
title: 一般グラフの最小カット（Stoer–Wagner）
documentation_of: //graph_flow/stoer_wagner.hpp
---

# 一般グラフの最小カット（Stoer–Wagner）

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#stoer_wagner) / [実装](../../graph_flow/stoer_wagner.hpp) / [一次資料と設計](../b_7_11/references.md)

最大隣接度探索のphaseごとに最後の頂点が作るcutを取り出し、最後の2頂点を縮約します。

## usage

```cpp
#include <iostream>
#include "graph_flow/stoer_wagner.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto cut=cp::stoer_wagner(g); std::cout << cut->cost << '\n';
}
```

出力:

```text
3
```

## API・使用条件

`stoer_wagner(g)` → optional<GlobalMinCutResult<Cap>>（cost,side）。非負整数重み、64bit以下。無向、多重辺可、ループ無視。V<2はnullopt。

有向グラフ・負の重みは`invalid_argument`、最小カット値が Cap に収まらなければ`overflow_error`です。

## 仕組みと境界

sideとその補集合は両方非空。非連結なら費用0のcutも返します。内部の辺和は128bit、返す費用はCapへ検査して変換します。

## 計算量

時間O(V³+E)、領域O(V²)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: グラフを2つに分けるために切る辺の重みの最小値（全体最小カット）と、その分割。 [実行できる小問題](../../examples/graph_flow/stoer_wagner_b.cpp)。

検証では、push-relabel と同じ800例（ループ・多重辺あり）で全ての 2^V−2 通りの分割の最小値と一致し、`side` の両側が空でなく同じカット値を持つことを確かめます。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/flow_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
