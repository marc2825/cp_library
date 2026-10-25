---
title: 二端子直並列グラフ分解
documentation_of: //graph_combinatorics/series_parallel_decomposition.hpp
---

# 二端子直並列グラフ分解

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#series_parallel_decomposition) / [実装](../../graph_combinatorics/series_parallel_decomposition.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

特殊な分解の対象を有向二端子直並列グラフと定め、series/parallel合成の式木を復元します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/series_parallel_decomposition.hpp"
int main() {
    cp::Graph<int> g(4,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,3); g.add_edge(0,2); g.add_edge(2,3);
    auto r=cp::series_parallel_decomposition(g,0,3);
    std::cout << r->nodes.size() << '\n';
}
```

出力:

```text
7
```

## API・使用条件

`series_parallel_decomposition(g,source,sink)`→optional<Result>。`root`と `nodes`。Nodeはkind=edge/series/parallel、from/to端子、left/right子、葉のoriginal_edge。

0始まりの異なる有効端子が必要。向きはsource→sinkに従う有向グラフ。平行辺可、ループ・孤立点・余分な成分・直並列でない構造はnullopt。不正端子/無向はinvalid_argument。全元頂点と元辺を使う定義です。

## 仕組み

同じ端点の平行辺をまとめ、内部の入次数1・出次数1の頂点をseriesに縮約します。更新した端点だけをqueueに戻します。最終的にsource→sinkの1辺になれば成功し、縮約で作った二分式木が構成の証拠です。

## 計算量

O((n+m)log(n+m+1))時間、O(n+m)領域。成功時、m>=1の式木ノード数は2m−1。

## 使用する問題・検証

直列/並列に組まれた有向回路、二端子ネットワークのDP、合成履歴の復元。無向SPQRとは別の仕様です。

250小グラフの辺部分集合からseries/parallel式を構成する独立DPと比較。葉の辺ID一意性と端子合成を検査。 [実行できる使用例](../../examples/graph_combinatorics/series_parallel_decomposition.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
