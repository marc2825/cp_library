---
title: 一般グラフ最大重みMatching
documentation_of: //graph_flow/general_weighted_matching.hpp
---

# 一般グラフ最大重みMatching

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#general_weighted_matching) / [実装](../../graph_flow/general_weighted_matching.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

一般無向グラフの重み付きBlossomで、辺重み総和を最大化するmatchingを返します。

## usage

```cpp
#include <iostream>
#include "graph_flow/general_weighted_matching.hpp"
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0,1,5); g.add_edge(1,2,7); g.add_edge(2,0,4);
    auto r=cp::general_weighted_matching(g);
    std::cout << cp::optimization_to_string(r.weight) << ' ' << r.edge_ids.size() << '\n';
}
```

出力:

```text
7 1
```

## API・使用条件

`general_weighted_matching(g)`→`GeneralWeightedMatchingResult`。128bit `weight`、未対応−1の `mate`/`mate_edge`、採用した元 `edge_ids`。入力費用はbool以外の64bit以下整数。

空matchingを許し、重み総和のみ最大化します。最大本数の優先・同点の本数最大化は保証しません。0以下の辺とループは除き、同じ端点の辺は最大重み（同値なら最初）に集約。有向はinvalid_argument。Boost Graphヘッダが必要で追加リンク不要。

## 仕組み

Boost Graph 1.89の修正版Blossom coreを固定commitから取り込み、独自namespaceに隔離しました。128bit費用で内部の2倍/4倍演算を安全に扱い、元辺IDへ復元。環境のBoost 1.74のmatching本体には依存しません。著作権表示・BSL-1.0・出典hashを保存。

## 計算量

O(n³+m log(m+1))時間、O(n+m)領域。辺集約のmapを含む。

## 使用する問題・検証

一般グラフの相性・利益最大化、奇閉路を含む重み付きペア選択。 [General Weighted Matching](https://judge.yosupo.jp/problem/general_weighted_matching) の形式に接続できます。

2,500例をbitmask DPと比較。奇閉路・多重辺・負/0費用・64bit極値と復元を検査。LSanも有効にして所有権を確認。 [実行できる使用例](../../examples/graph_flow/general_weighted_matching.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
