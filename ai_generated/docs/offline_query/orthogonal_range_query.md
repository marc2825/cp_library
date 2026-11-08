---
title: 多次元直交範囲クエリ
documentation_of: //offline_query/orthogonal_range_query.hpp
---

# 多次元直交範囲クエリ（D-33）

3次元以上を含む固定次元の静的点集合について、軸平行な半開box内の重み和と点数を答えます。D-35の疎な多次元Fenwick Treeを共有し、重みと個数を別々に構築します。重複座標の点は別々に数えます。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#orthogonal_range_query)

## API・使用条件

`OrthogonalRangeQuery<T,D,Coordinate=long long>(points,weights)`、`(points={})` は各点重みT(1)。`sum(lower,upper)`, `count(lower,upper)`。点・境界は `array<Coordinate,D>`。

Dは1..10。各軸でlower以上upper未満。空・逆向きboxは0。重みは可換加法群（T(0), +=, -= または二項差）、重み長の不一致はinvalid_argument。静的APIのため構築後の更新はなく、更新にはD-35を直接使います。

## 計算量・保存領域

固定次元Dで構築 O(n log^D(n+1))、問い合わせ O(2^D log^D(n+1))、領域 O(n log^(D-1)(n+1)) の上界。次元の指数係数が大きいため低次元向けです。

## 使用例

[コンパイル可能な使用例](../../examples/offline_query/orthogonal_range_query.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../offline_query/orthogonal_range_query.hpp"
int main(){
    using Query=cp::OrthogonalRangeQuery<long long,3>;
    Query query({{0,0,0},{1,2,3},{1,2,3}}, {4,5,6});
    std::cout<<query.count({0,0,0},{2,3,4})<<' '<<query.sum({1,2,3},{2,3,4})<<'\n';
}
```

```text
3 11
```

## 使用する問題

静的3D/4D rectangle sum、属性の多条件集計。列挙・最小値を返すAPIではありません。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Fenwick, A New Data Structure for Cumulative Frequency Tables](https://doi.org/10.1002/spe.4380240306) のprefix構造を再帰的に使い、boxは包除で求めます。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
