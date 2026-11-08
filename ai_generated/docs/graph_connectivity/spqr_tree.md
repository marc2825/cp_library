---
title: SPQR Tree（分離対探索版）
documentation_of: //graph_connectivity/spqr_tree.hpp
---

# SPQR Tree（分離対探索版）

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#spqr_tree) / [実装](../../graph_connectivity/spqr_tree.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

二重頂点連結グラフをS（cycle）・P（parallel）・R（三重頂点連結）スケルトンへ分解します。線形時間版ではなく、分離対を列挙する実装です。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/spqr_tree.hpp"
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(0,1);
    auto r=cp::spqr_tree(g);
    std::cout << r->nodes.size() << ' ' << r->tree.edge_count() << '\n';
}
```

出力:

```text
2 1
```

## API・使用条件

`spqr_tree(g)`→optional<SPQRTree>。`nodes[].kind/edges`、分解木 `tree`、`original_location[edge]=(node,local_edge)`。スケルトン辺は元頂点ID `from/to`、実辺の `original_edge` または `virtual_id` と `twin_node/twin_edge`。

ループなし無向多重グラフ。n<2、非連結、関節点ありはnullopt。ループ/有向入力はinvalid_argument。Qノードは単一辺の場合のみで、通常の実辺のQは省略。2頂点の複数辺はP。根は固定しません。仮想IDには併合で消えた欠番があり、連番を要求しないでください。

## 仕組み

各頂点対を除いた成分と端点間の直接辺をsplit componentとします。2つの非自明成分なら双対の仮想辺で分割し、3成分以上なら中央Pへ接続します。最後に隣接するS-S/P-Pを併合。すべての実辺は1回、仮想辺は対で保存します。

## 計算量

最悪O(m n²(n+m))時間、作業・出力O(n+m)領域。小中規模の検証しやすい分離対探索版です。線形時間のHopcroft–Tarjan法の性能は保証しません。

## 使用する問題・検証

二点除去による分離、二重頂点連結グラフの直列・並列・剛体部分の構造解析。

5頂点以下の全単純グラフと250多重グラフ。各Rの二点除去連結性、S/P形状、仮想辺対、実辺一意性、分解木と実辺集合の分離を検査。 [実行できる使用例](../../examples/graph_connectivity/spqr_tree.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
