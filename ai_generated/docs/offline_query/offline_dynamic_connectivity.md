---
title: Offline Dynamic Connectivity
documentation_of: //offline_query/offline_dynamic_connectivity.hpp
---

# Offline Dynamic Connectivity

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#connectivity) / [実装](../../offline_query/offline_dynamic_connectivity.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "offline_query/offline_dynamic_connectivity.hpp"
int main() {
    cp::OfflineDynamicConnectivity g(3); g.add_edge(0, 1); g.add_edge(1, 2);
    g.add_query(0, 2); g.remove_edge(1, 2); g.add_query(0, 2);
    for (bool b : g.solve()) std::cout << b << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/offline_query/offline_dynamic_connectivity.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`OfflineDynamicConnectivity(n)`。`add_edge(u,v)`、`remove_edge(u,v)->bool`、`add_query(u,v)->int`、`query_count()`、`solve() const -> vector<bool>`。無向グラフで多重辺・自己ループ可。辺の重複数が正の間は存在します。存在しない辺の削除はfalseで操作を登録しません。質問IDは登録順、solveは追記前後や複数回呼出し可。n=0では辺・質問を登録できずsolveは空。

## 計算量・原理

登録操作数Q・頂点数nとしてO(Q log Q log(n+1)+n)時間、O(n+Q log Q)領域。mapの登録処理はO(log Q)。

各辺の有効期間[l,r)を時間セグメント木へ分配し、DFSでその時間帯に共通する辺をRollback DSUに加えます。枝を出るときに履歴を戻し、削除を含む時系列の各質問を処理します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/dynamic_graph_vertex_add_component_sum)は頂点値の加算と成分和も必要で、連結判定だけを返すこの API 単体では解けません。[SegmentTreeOverTime](segment_tree_over_time.md)と`union_find/rollback_disjoint_set_union_with_data.hpp`を直接組み合わせてください。

[追加分の検証入口](../../verify/run_advanced.py)で、n≤15・400操作のランダムな多重グラフの履歴120個について、100操作ごとの`solve`・再実行・存在しない辺の`remove_edge`（false）・質問IDをBFSと比較します（多重辺の扱いは`verify/data_structure_extensions/rollback_test.cpp`でも確認）。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。

2026-10-24から、期間登録とDFSは[汎用SegmentTreeOverTime](segment_tree_over_time.md)を共有しています。add/remove/query/solveの従来の契約は保持します。
