---
title: Parallel Union-Find
documentation_of: //union_find/parallel_disjoint_set_union.hpp
---

# Parallel Union-Find

同じ長さの二つの区間を、対応するoffset同士でまとめて併合します。長さ2^kごとのDSUで既に処理した区間対を飛ばし、初めての併合だけを下位へ伝えます。

## 使用例

```cpp
#include <iostream>
#include "union_find/parallel_disjoint_set_union.hpp"
int main() {
    cp::ParallelDisjointSetUnion d(8); d.merge_ranges(0,4,3);
    std::cout << d.same(1,5) << " " << d.same(3,7) << "\n";
}
```

出力: `1 0`。

## APIと使用条件

`ParallelDisjointSetUnion(n)`、merge_ranges(a,b,length)、merge(u,v)、leader/same/size/component_count。

区間は配列[0,N)内、長さ0可。ここでParallelは区間同士の一括結合であり、マルチスレッド実装ではありません。通常mergeとの混在も可。

## 計算量

構築・領域O(N log N)。Q回の区間併合全体でO((N log N+Q) α(N))、単点検索は償却O(α(N))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

区間同士の同一視、オンライン辺追加、頂点群への一括加算、辺の削除を含む連結性などで使います。成分ラベルの直接更新、全辺を除いたBFSと比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#parallel_disjoint_set_union) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#dsu) / [図解の簡略化](../../visualizations/b_1_6/README.md)
