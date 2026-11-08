---
title: 動的単位容量Max Flow
documentation_of: //graph_flow/dynamic_unit_flow.hpp
---

# 動的単位容量Max Flow

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#dynamic_unit_flow) / [実装](../../graph_flow/dynamic_unit_flow.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

固定した始点・終点に対する有向単位容量最大流を、辺追加・有効化・無効化のたびに更新します。

## usage

```cpp
#include <iostream>
#include "graph_flow/dynamic_unit_flow.hpp"
int main() {
    cp::DynamicUnitFlow f(3,0,2);
    int e=f.add_edge(0,1); f.add_edge(1,2);
    std::cout << f.value() << ' ' << f.set_enabled(e,false) << ' ';
    f.add_edge(0,2); std::cout << f.value() << '\n';
}
```

出力:

```text
1 0 1
```

## API・使用条件

`DynamicUnitFlow(n,s,t)`、`add_edge(u,v,enabled=true)`→不変辺ID、`set_enabled(id,bool)`→更新後の最大流、`value()`、`edge_count()`、`get_edge(id)`→from/to/capacity/flow、`min_cut()`→始点側bool列。

nとs,tは固定でs!=t。容量は0/1だけ。平行辺・ループ可。消したIDは再利用せず再有効化可能。不正サイズ・端点・IDは例外。min_cutは更新直後の最大流に対応。

## 仕組み

追加で増える流量は高々1なのでBFSを1回行います。使用中の辺を消すときは残余網で端点間をつなぎ直します。補助のt→s帰還辺を使って必要なら流量を1下げ、最後にs→tの増加路を再探索します。

## 計算量

各更新O(n+m)、領域O(n+m)。value/get_edgeはO(1)、min_cutはO(n+m)。mは無効化済みも含めた登録辺数です。

## 使用する問題・検証

利用可能なリンクが開閉するネットワークの辺素パス本数。任意容量の動的フローは対象外です。

18,000更新について全s-tカット列挙と比較し、各辺の容量・保存則・返却min-cutも検査。 [実行できる使用例](../../examples/graph_flow/dynamic_unit_flow.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
