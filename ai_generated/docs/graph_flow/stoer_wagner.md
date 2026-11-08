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

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

sideとその補集合は両方非空。非連結なら費用0のcutも返します。内部の辺和は128bit、返す費用はCapへ検査して変換します。

## 計算量

時間O(V³+E)、領域O(V²)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 最大隣接度探索のphaseごとに最後の頂点が作るcutを取り出し、最後の2頂点を縮約します。 [実行できる小問題](../../examples/graph_flow/stoer_wagner_b.cpp)。

一般グラフの最小カット（Stoer–Wagner）を必要とする3.9の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
