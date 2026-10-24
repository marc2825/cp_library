---
title: Sparse Multidimensional Fenwick Tree
documentation_of: //range_query/fenwick_tree_nd.hpp
---

# Sparse Multidimensional Fenwick Tree（D-35）

更新する座標だけを事前登録する固定次元の疎なFenwick Treeです。各軸のFenwickノード内に次元を1つ減らした木を置き、1次元で既知のprefix集約へ帰着します。全座標の直積領域は確保しません。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#fenwick_tree_nd)

## API・使用条件

`FenwickTreeND<T,D,Coordinate=long long>(points={})`、Pointはarray<Coordinate,D>。

- `add(point,delta)` は登録点に加算。
- `prefix_sum(upper)` は全軸でupper未満の和。
- `sum(lower,upper)` は各軸lower以上upper未満のbox和。
- `contains(point)` は登録の有無、`point_count()` は異なる登録座標数。

Dは1..10、重みは可換加法群（T(0), +=, 二項差）。Coordinateはデフォルト構築・全順序・等値比較可能。未登録点のaddはinvalid_argumentで変更なし。問い合わせ境界は登録不要です。箱が空・逆向きなら0。算術途中値もTに収まること。

## 計算量・保存領域

固定Dで構築 O(n log^D(n+1))、点加算・prefix和 O(log^D(n+1))、box和 O(2^D log^D(n+1))、領域 O(n log^(D-1)(n+1)) の上界。登録点の重複は取り除きます。

## 使用例

[コンパイル可能な使用例](../../examples/range_query/fenwick_tree_nd.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../range_query/fenwick_tree_nd.hpp"
int main(){
    cp::FenwickTreeND<long long,3> tree({{1,2,3},{4,5,6}});
    tree.add({1,2,3},7); tree.add({4,5,6},11);
    std::cout<<tree.prefix_sum({5,6,7})<<' '<<tree.sum({0,0,0},{3,4,5})<<'\n';
}
```

```text
18 7
```

## 使用する問題

疎な3D/4D point-add rectangle-sum。静的点数・重みならD-33のラッパーも使えます。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Fenwick, A New Data Structure for Cumulative Frequency Tables](https://doi.org/10.1002/spe.4380240306)。座標圧縮したFenwickノードを次元方向に再帰化した独自実装です。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
