---
title: 動的木直径
documentation_of: //tree/dynamic_tree_diameter.hpp
---

# 動的木直径

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dynamic_tree_diameter) / [実装](../../tree/dynamic_tree_diameter.hpp) / [一次資料と設計](../b_7_11/references.md)

Static Top Treeのクラスタに両端からの最遠頂点と内部直径を持たせ、辺重み更新後の直径を計算します。

## usage

```cpp
#include <iostream>
#include "tree/dynamic_tree_diameter.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::DynamicTreeDiameter<> d(g); d.set_edge(0,7);
    std::cout << d.diameter()->length << '\n';
}
```

出力:

```text
10
```

## API・使用条件

`DynamicTreeDiameter<T>(g)`、`set_edge(id,w),component_diameter(v),diameter()`。無向森、非負辺。結果{length,first,second}。diameter()は空ならnullopt。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

固定topologyの重み更新を扱います。link/cutを含む直径更新はC項目のDynamic Top Tree側へ残します。長さ0の自己パスも許し、同値の端点対は辞書順最小。距離の全中間値をTに収める必要があります。負重みはinvalid_argument。

## 計算量

構築O(V)、辺更新O(log²(V+1))、成分直径O(1)、森全体diameter()は成分数Cに対しO(C)、領域O(V)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: Static Top Treeのクラスタに両端からの最遠頂点と内部直径を持たせ、辺重み更新後の直径を計算します。 [実行できる小問題](../../examples/tree/dynamic_tree_diameter_b.cpp)。

動的木直径を必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
