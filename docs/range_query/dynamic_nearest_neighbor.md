---
title: Dynamic Multidimensional Nearest Neighbor
documentation_of: //range_query/dynamic_nearest_neighbor.hpp
---

# Dynamic Multidimensional Nearest Neighbor（D-34）

要仕様化だったD-34を「1〜10次元の点追加・削除・移動と二乗Euclid最近傍」と定義しました。KD-treeの静的部分と更新bufferを平方分割で管理し、定期的に再構築します。移動前の座標は世代番号で無効化し、同距離なら小さいIDを返します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#dynamic_nearest_neighbor)

## API・使用条件

`DynamicNearestNeighbor<D>()` / `(vector<Point>)`、Pointはarray<long double,D>。

- `insert(point)` → 再利用しないID、`erase(id)` / `move(id,point)` → 活動中ならtrue。
- `nearest(point)` → `optional<Result>{id,squared_distance}`、空集合なら空。
- `point(id)`, `contains(id)`, `size()`, `empty()`, `rebuild()`。

座標は有限値、NaN/infはinvalid_argument。距離はlong doubleの演算結果で比較し、厳密な実数幾何ではありません。差・二乗和が有限に収まる範囲の利用を推奨します。削除済みpoint(id)はout_of_range。過去ID記録は残るため、削除しても累計ID領域は減りません。世代番号は再構築時にリセットします。

## 計算量・保存領域

累計発行ID数M、現存点数nに対し、再構築 O(M+n log(n+1))、更新はならし O(√(M+1) log(M+1)) の上界。問い合わせはKD探索とbuffer走査、最悪 O(n+√(M+1))。領域 O(M+n)。次元Dは定数として数えます。高次元での多項対数探索保証はありません。

## 使用例

[コンパイル可能な使用例](../../examples/range_query/dynamic_nearest_neighbor.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../range_query/dynamic_nearest_neighbor.hpp"
int main(){
    cp::DynamicNearestNeighbor<3> points;
    int a=points.insert({0,0,0}),b=points.insert({3,0,0});
    std::cout<<points.nearest({2,0,0})->id<<'\n';
    points.move(a,{2,0,0}); points.erase(b);
    auto result=*points.nearest({2,1,0});
    std::cout<<result.id<<' '<<result.squared_distance<<'\n';
}
```

```text
1
0 1
```

## 使用する問題

点集合の移動を含む最近傍、低次元のオンライン座標検索。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Bentley, Multidimensional Binary Search Trees](https://doi.org/10.1145/361002.361007)。KD-treeのbounding box探索と平方分割による再構築を独自に組み合わせました。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
