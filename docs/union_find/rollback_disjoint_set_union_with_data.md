---
title: DSU callback / Rollback data
documentation_of: //union_find/rollback_disjoint_set_union_with_data.hpp
---

# DSU callback / Rollback data（D-10）

通常 DSU と Rollback DSU に併合 callback を追加し、任意の付随情報を同時に巻き戻すラッパーを用意しました。付随情報は成分集約だけに限定しません。

## API・使用条件

`DisjointSetUnion::merge(u,v,callback)` と `RollbackDisjointSetUnion::merge(u,v,callback)` は、異なる成分のときだけ callback(残る根,吸収される根) を構造変更直前に呼びます。union by size、同サイズなら第1引数側を残します。旧2引数 merge はそのまま使えます。callback が投げたとき分割は変わらず、Rollback DSU の merge 履歴も追加しません。通常 DSU の経路圧縮は起こり得ます。

`RollbackDisjointSetUnionWithData<Data>(n,data)` は外部 Data を借用します。Data は const snapshot() と rollback(token) を持ち、ラッパーより長く生存すること。データ更新には必ず Data 自身の履歴に記録する API を使います。既存 RollbackArray とそのまま組み合わせられます。コピー・ムーブ不可。

`merge(u,v,callback)` の callback は (data,残る根,吸収される根)。snapshot()/rollback(token)、undo()、leader/same/component_size/component_count/size、data() を公開します。merge は失敗（同じ成分）でも undo の履歴1件。undo は直前の merge より前のデータへ戻すため、その merge の後に data() で行った変更も戻します。snapshot は merge 間のデータ編集も含めて保存します。

snapshot は所有者と DSU 分岐履歴を検証し、別個体・破棄した枝・未来の履歴は invalid_argument。Data の履歴妥当性は Data 側も検証すること。保存点は現在の履歴上の祖先のみ有効です。DSU の変更がない枝は Data の token が区別します。

callback の途中例外では Data を merge 前へ戻し、DSU も変更しません。rollback(valid token) は例外を投げないこと。Data を直接 rollback/clear_history してラッパーの履歴を無効化しないでください。callback 中に同じ DSU を再帰的に更新したり、履歴外の外部状態を復元対象だと見なしたりしないでください。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include <vector>
#include "basic_utility/rollback_array.hpp"
#include "union_find/rollback_disjoint_set_union_with_data.hpp"
int main() {
    cp::RollbackArray<int> sums(std::vector<int>{2,3,7});
    cp::RollbackDisjointSetUnionWithData<cp::RollbackArray<int>> dsu(3,sums);
    auto saved=dsu.snapshot();
    dsu.merge(0,1,[](auto& data,int kept,int absorbed){ data.set(kept,data[kept]+data[absorbed]); });
    std::cout << sums[dsu.leader(1)] << " " << dsu.component_count() << "\n";
    dsu.rollback(saved);
    std::cout << sums[0] << " " << sums[1] << " " << dsu.component_count() << "\n";
}
```

```text
5 2
2 3 3
```

## 計算量

通常 DSU は償却 O(α(n)) + callback。Rollback DSU は leader/merge O(log(n+1)) + callback、undo は O(1) + Data の復元コスト。ラッパーの snapshot は O(1) + Data snapshot、rollback は取り消す merge 数 + Data の復元コスト。領域 O(n+merge 履歴数) + Data の領域。

## 問題例・検証・参考資料

例題: 動的連結成分ごとの総和・種類数・矛盾数。8万の分岐操作で素朴な成分ラベルと総和を比較。失敗 merge の undo、任意の追加データ、別個体/古い枝 token、途中例外・callback の根順序・通常 DSU 互換を検証します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#rollback_disjoint_set_union_with_data) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/union_find/rollback_disjoint_set_union_with_data.cpp) / [カテゴリ索引](README.md)
