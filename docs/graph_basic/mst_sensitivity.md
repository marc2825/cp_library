---
title: 最小全域木の感度解析
documentation_of: //graph_basic/mst_sensitivity.hpp
---

# 最小全域木の感度解析

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#mst_sensitivity) / [実装](../../graph_basic/mst_sensitivity.hpp) / [一次資料と設計](../b_7_11/references.md)

一つのKruskal木を基準に、各辺を必ず使う／禁止する場合の最小費用と交換辺を求めます。HLDのパス最大値とパスchminを共有します。

## usage

```cpp
#include <iostream>
#include "graph_basic/mst_sensitivity.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto result=cp::mst_sensitivity(g);
    std::cout << *result->forced_cost[2] << ' ' << *result->forbidden_cost[1] << '\n';
}
```

出力:

```text
5 5
```

## API・使用条件

`mst_sensitivity(g)` → `optional<MSTSensitivityResult<T>>`。無向、負辺可、非連結ならnullopt。`cost,in_tree,forced_cost,forbidden_cost,replacement_edge,removed_edge,different,strictly_larger`。各optional費用がnulloptなら該当する木なし。交換は{cost,added_edge,removed_edge}。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

木辺を除いたときの最安代替辺を、非木辺のパス全体へのchminで求めます。自己ループは強制不可。distinctな最大2重みを記録し、等費用の別MSTと厳密に費用が増える次の木を区別します。HLDの根距離・差分・中間値もTに収まることが必要です。

## 計算量

時間O((V+E) log²(V+1)+E log(E+1))、領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 一つのKruskal木を基準に、各辺を必ず使う／禁止する場合の最小費用と交換辺を求めます。HLDのパス最大値とパスchminを共有します。 [実行できる小問題](../../examples/graph_basic/mst_sensitivity_b.cpp)。

最小全域木の感度解析を必要とする3.7の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
