---
title: 一般グラフの木幅分解
documentation_of: //graph_connectivity/tree_decomposition.hpp
---

# 一般グラフの木幅分解

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#tree_decomposition) / [実装](../../graph_connectivity/tree_decomposition.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

任意幅の最小木幅分解と、大きい入力にも使えるmin-fillヒューリスティックを別APIで提供します。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/tree_decomposition.hpp"
int main() {
    cp::Graph<int> g(4);
    for(int v=0;v<4;++v) g.add_edge(v,(v+1)%4);
    auto r=cp::minimum_tree_decomposition(g);
    std::cout << r.width << ' ' << r.bags.size() << '\n';
}
```

出力:

```text
2 4
```

## API・使用条件

`minimum_tree_decomposition(g)`はn<=22の厳密最適解。`greedy_tree_decomposition(g)`はサイズ制限を置かない上界。Resultの `width`, `optimal`, `bags`（ID=消去元頂点）、`elimination_order`, `forest`。

無向、非連結可。ループは無視し平行辺は単純化します。空グラフのwidth=−1。厳密版のn>22はlength_error。有向はinvalid_argument。greedyのoptimalは常にfalse。forestは成分ごとの分解で、全体を1本にする必要がある利用者は別成分のbagを任意に接続できます。

## 仕組み

厳密版は消去済み集合Sのsubset DP。vからSの頂点だけを内部に通って到達する未消去頂点が、消去時のfill-neighborです。幅の最大値を最小化する順序を復元します。greedyは追加fill辺数を最小化する頂点を選びます。後の隣接集合をbagにし最初に消える後続頂点のbagへ接続します。

## 計算量

厳密版O(n²2^n)時間・O(2^n+n²)領域。greedy O(n⁴)時間・O(n²)領域。n=22はDPだけで約8MiBですが計算量は大きいため、22は高速性の保証でなくハード上限です。

## 使用する問題・検証

小グラフの最小木幅、任意幅のbag DPの前処理。既存width-2専用APIも保持。

5頂点以下全グラフの全消去順序と比較。bagによる辺被覆、頂点bagの連結性、順序の幅、greedy上界を検査。 [実行できる使用例](../../examples/graph_connectivity/tree_decomposition.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
