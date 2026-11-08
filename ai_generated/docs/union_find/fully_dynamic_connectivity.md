---
title: Fully Dynamic Connectivity
documentation_of: //union_find/fully_dynamic_connectivity.hpp
---

# Fully Dynamic Connectivity（D-21）

一般の無向グラフに対し、オンラインで辺を追加・削除します。全域森を既存Euler Tour Treeに保存し、木辺の削除時は小さい側の成分を探索して代替辺を探します。多重辺・自己ループにも別々のIDを発行します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#fully_dynamic_connectivity)

## API・使用条件

`FullyDynamicConnectivity(n)`。

- `add_edge(u,v)` → 再利用しない辺ID。
- `remove_edge(id)` → 削除できたときtrue、無効・削除済みはfalse。
- `active(id)`, `connected(u,v)`, `component_size(v)`, `component_count()`, `size()`。
- `compact_forest()` でETTの過去のlinkノードを解放します。

頂点数は固定、頂点添字はassert。辺IDの記録はcompact後も残ります。オンライン処理を優先した実用的な走査方式です。非常に多い木辺削除では線形走査が支配的です。

## 計算量・保存領域

累計辺ID数をMとします。連結・成分サイズは期待 O(log n)、辺追加は期待 O(log n+log(M+1))。非木辺の削除は集合操作を含め O(log(M+1))。木辺の削除は小さい側の頂点数 s とそこから調べる辺数 a に対して O(s+a+log(M+1)+log n)。HDTの多項対数時間保証は持ちません。領域は頂点数・累計辺ID数・ETTの累計link数に線形。`compact_forest()` は生存木辺だけでETTを再構築し O(M+n log n) です。

## 使用例

[コンパイル可能な使用例](../../examples/union_find/fully_dynamic_connectivity.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../union_find/fully_dynamic_connectivity.hpp"
int main(){
    cp::FullyDynamicConnectivity graph(4);
    int a=graph.add_edge(0,1); graph.add_edge(1,2);
    int c=graph.add_edge(0,2); graph.remove_edge(a);
    std::cout<<graph.connected(0,2)<<' '<<graph.component_size(0)<<'\n';
    graph.remove_edge(c);
    std::cout<<graph.connected(0,2)<<' '<<graph.component_count()<<'\n';
}
```

```text
1 3
0 3
```

## 使用する問題

辺の出入りを逐次処理する無向グラフの連結判定。オフラインでよければ既存OfflineDynamicConnectivityも選べます。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Henzinger–King, dynamic graph algorithms](https://doi.org/10.1145/293347.293349) の全域森と代替辺という考え方を参照。ここでは既存Euler Tour Treeと小成分走査を組み合わせます。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
