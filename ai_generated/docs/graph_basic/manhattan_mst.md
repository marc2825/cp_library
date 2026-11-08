---
title: Manhattan MST
documentation_of: //graph_basic/manhattan_mst.hpp
---

# Manhattan MST

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#manhattan_mst) / [実装](../../graph_basic/manhattan_mst.hpp) / [一次資料と設計](../b_7_11/references.md)

平面上の点のL1距離について、4方向のsweepでO(V)本の候補辺を作り、Kruskalで全域木を構成します。

## usage

```cpp
#include <iostream>
#include "graph_basic/manhattan_mst.hpp"
int main() {
    auto mst=cp::manhattan_mst({{0,0},{2,0},{2,3}});
    std::cout << mst.cost << '\n';
}
```

出力:

```text
5
```

## API・使用条件

`manhattan_mst(vector<pair<long long,long long>> points)` → `MinimumSpanningForestResult<long long>`。重複座標・負座標可。返す辺の端点は入力点の添字、辺IDは候補グラフ内のID。空入力可。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

座標変換・比較には符号付き128bitを使い、LLONG_MINの反転も安全に行います。候補の距離と総費用はsigned64に収まる必要があります。KACTLのCC0実装を参考に移植し、ヘッダに出典を記載。

## 計算量

時間O(V log(V+1))、領域O(V)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 平面上の点のL1距離について、4方向のsweepでO(V)本の候補辺を作り、Kruskalで全域木を構成します。 [実行できる小問題](../../examples/graph_basic/manhattan_mst_b.cpp)。

関連する出題: [Manhattan MST](https://judge.yosupo.jp/problem/manhattanmst)。入出力仕様の参照先であり、オンラインACの主張ではありません。

Manhattan MSTを必要とする3.7の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
