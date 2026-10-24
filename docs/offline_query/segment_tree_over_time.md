---
title: Segment Tree over Time
documentation_of: //offline_query/segment_tree_over_time.hpp
---

# Segment Tree over Time（D-09）

各操作の有効期間を [l,r) で登録し、各時刻の状態を snapshot/apply/rollback で構築します。DSU 以外の巻き戻せる構造にも使用できます。OfflineDynamicConnectivity はこの基盤を共有します。

## API・使用条件

`SegmentTreeOverTime<Event>(times)`、`add(l,r,event)`、`size()`、`event_count()`。

`run(state,apply,visit)` は apply(state,const Event&) と visit(state,int time) を呼びます。`run(state,visit)` は state.apply(event) を利用します。State は snapshot() と rollback(token) を持ち、token は値として保存可能であること。Event は移動可能でよく、コピー不要。イベント本体は1個、標準ノードには ID のみを置きます。空期間は登録しません。

visit は時刻 0..times-1 の順に1回ずつ呼び、各葉で有効な登録をそれぞれ1回適用します。適用順序は根から葉へのノード順、同じノードでは登録順です。**全体の登録順・時刻順に apply する契約ではありません。** 加算や辺の併合など、求める答えが適用順に依存しない用途に使用してください。

run 後は呼出し前の状態に復元し、繰り返し利用できます。apply/visit が例外を投げても各階層を戻して再送出します。rollback(valid token) は例外を投げず、途中まで変更した apply も復元できることが必要です。visit が変更した外部出力は復元しません。run 中に timeline や保存済み履歴を外部から変更しないでください。times=0 は訪問0回。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include <utility>
#include "offline_query/segment_tree_over_time.hpp"
#include "union_find/rollback_disjoint_set_union.hpp"
int main() {
    cp::SegmentTreeOverTime<std::pair<int,int>> timeline(4);
    timeline.add(0,3,{0,1}); timeline.add(1,2,{1,2});
    cp::RollbackDisjointSetUnion dsu(3);
    timeline.run(dsu,[](auto& d,const auto& e){ d.merge(e.first,e.second); },
        [](auto& d,int t){ std::cout << t << ": " << d.component_count() << "\n"; });
    std::cout << "restored " << dsu.component_count() << "\n";
}
```

```text
0: 2
1: 1
2: 2
3: 3
restored 3
```

## 計算量

T=max(1,times)、M は非空イベント数。構築 O(T)、add は O(log T) 時間・ID 数、全領域 O(T+M log T)。run は O(T+M log T) 回程度の snapshot/apply/rollback/visit と O(log T) の再帰補助領域。各操作のコストは State に依存します。

## 問題例・検証・参考資料

例題: 動的連結性、期間限定の加算・制約。101種類の時刻長について有効辺から BFS で全成分と総和を比較。繰り返し実行、非空の初期状態、apply/visit 例外復元、move-only Event を検証します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#segment_tree_over_time) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/offline_query/segment_tree_over_time.cpp) / [カテゴリ索引](README.md)
