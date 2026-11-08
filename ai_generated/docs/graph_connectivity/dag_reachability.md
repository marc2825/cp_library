---
title: DAG Reachability
documentation_of: //graph_connectivity/dag_reachability.hpp
---

# DAG Reachability

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dag_reachability) / [実装](../../graph_connectivity/dag_reachability.hpp) / [一次資料と設計](../b_7_11/references.md)

逆topological順でpacked bitsetを併合する全対版と、64問い合わせずつ始点bitを伝える省領域バッチ版を用意します。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/dag_reachability.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2);
    cp::DAGReachability r(g); std::cout << r.reachable(0,2) << ' ' << r.reachable(2,0) << '\n';
}
```

出力:

```text
1 0
```

## API・使用条件

`DAGReachability(g).reachable(u,v)`。`dag_reachability(g,vector<pair<int,int>> queries)` → vector<bool>。有向DAG以外はinvalid_argument。長さ0の道を含みu=uはtrue。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

到達集合を保持する方向と、問い合わせの始点bitを伝える方向は逆です。空入力・空問い合わせ可。重みは無視します。グラフの更新は反映しません。

## 計算量

全対構築O((V+E) ceil(V/64))、領域O(V ceil(V/64))、query O(1)。バッチO((V+E) ceil(Q/64)+Q)、領域O(V+E+Q)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 逆topological順でpacked bitsetを併合する全対版と、64問い合わせずつ始点bitを伝える省領域バッチ版を用意します。 [実行できる小問題](../../examples/graph_connectivity/dag_reachability_b.cpp)。

DAG Reachabilityを必要とする3.8の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
