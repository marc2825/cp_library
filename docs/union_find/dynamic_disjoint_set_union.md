---
title: 動的Union-Find
documentation_of: //union_find/dynamic_disjoint_set_union.hpp
---

# 動的Union-Find

頂点のラベルが事前に分からない場合に、検索・併合時に頂点を追加します。ラベル辞書と配列DSUを組み合わせます。

## 使用例

```cpp
#include <iostream>
#include "union_find/dynamic_disjoint_set_union.hpp"
#include <string>
int main() {
    cp::DynamicDisjointSetUnion<std::string> d; d.merge("alice","bob");
    std::cout << d.size("alice") << "\n";
}
```

出力: `2`。

## APIと使用条件

`DynamicDisjointSetUnion<Key,Compare>`、add(key)は安定ID、contains、merge/same/size、vertex_count/component_count。

same/sizeも未登録ラベルを追加します。containsは追加しません。辺削除を伴うFully Dynamic Connectivityではなく、頂点集合の動的拡張です。

## 計算量

ラベル検索O(log N)とDSU償却O(α(N))、領域O(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

区間同士の同一視、オンライン辺追加、頂点群への一括加算、辺の削除を含む連結性などで使います。成分ラベルの直接更新、全辺を除いたBFSと比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#dynamic_disjoint_set_union) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#dsu) / [図解の簡略化](../../visualizations/b_1_6/README.md)
