---
title: 加算操作付きDSU
documentation_of: //union_find/additive_disjoint_set_union.hpp
---

# 加算操作付きDSU

成分の加算tagと、併合前の加算履歴を打ち消すoffsetを保持します。点固有の加算は別に管理し、代表頂点への加算が他の頂点へ波及しないようにします。

## 使用例

```cpp
#include <iostream>
#include "union_find/additive_disjoint_set_union.hpp"
int main() {
    cp::AdditiveDisjointSetUnion<long long> d(3); d.add_component(0,5); d.merge(0,1); d.add_component(1,2);
    std::cout << d.get(0) << " " << d.get(1) << "\n";
}
```

出力: `7 2`。

## APIと使用条件

`AdditiveDisjointSetUnion<T>(nまたはvector<T>)`、merge/same/leader/size/component_count、add_component(v,x)、add_vertex(v,x)、get(v)、component_sum(v)。

Tは加減算と整数からの構築・乗算を持つ可換加法群。mergeは既存の頂点値を保存し、将来の成分加算だけ共有します。中間演算はT内に収まること。

## 計算量

各操作は償却O(α(N))、構築・領域O(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

区間同士の同一視、オンライン辺追加、頂点群への一括加算、辺の削除を含む連結性などで使います。成分ラベルの直接更新、全辺を除いたBFSと比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#additive_disjoint_set_union) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#dsu) / [図解の簡略化](../../visualizations/b_1_6/README.md)
