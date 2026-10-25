---
title: Shortest Nonzero Path
documentation_of: //graph_basic/shortest_nonzero_path.hpp
---

# Shortest Nonzero Path

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#shortest_nonzero_path) / [実装](../../graph_basic/shortest_nonzero_path.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

群ラベルの積が単位元でない、頂点を重複しない最短パスを全終点について求めます。単なる積グラフ上のwalk探索ではありません。

## usage

```cpp
#include <iostream>
#include "graph_basic/shortest_nonzero_path.hpp"
struct Parity {
    using value_type=int;
    static int identity() { return 0; }
    static int op(int a,int b) { return a^b; }
    static int inverse(int a) { return a; }
};
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1,1); g.add_edge(1,2,1); g.add_edge(0,2,5);
    auto r=cp::shortest_nonzero_paths<Parity>(g,{1,1,1},0);
    std::cout << cp::optimization_to_string(*r.distance[2]) << ' ' << r.path(2).edge_ids.size() << '\n';
}
```

出力:

```text
5 1
```

## API・使用条件

`shortest_nonzero_paths<Group>(g,labels,source=0)`→`ShortestNonzeroPathResult`。Groupは `value_type`, `identity()`, `op(a,b)`, `inverse(a)`と値の==。`distance[v]`はoptional<128bit>、`path(v)`は元頂点列 `vertices` と元辺列 `edge_ids`。

無向Graph、非負64bit以下整数費用。labelsは元辺のfrom→to方向、逆向きは群の逆元。非可換群にも対応。ループ・多重辺可。s→sの空パスは非零でないためnullopt、未到達もnullopt/空復元。不正型はコンパイル時、向き・ラベル数・費用・始点は例外。結果の復元用フィールドは変更しないでください。

## 仕組み

Iwata–YamaguchiのFastSUPを独立実装。まずDijkstraの木と木パスの群積を作ります。木と不整合な辺の高さを優先し、blossom内の未確定頂点をDSUで縮めます。整合辺にも非零パスからの高さを伝えます。各頂点に証拠辺と依存する非零パスを保存し、論文の2ケースに沿って単純パスを復元します。

## 計算量

計算O((n+m) log(n+m+1))、領域O(n+m)。群演算をO(1)として評価。復元は依存チェーンごとに列を組み直すため1終点あたり最悪O(n²)時間・O(n)追加領域。本実装は論文の線形時間復元まで最適化していません。

## 使用する問題・検証

最短の奇数辺パス、剰余和が0でない単純パス、群で表現する位相制約。

Z/3Zと非可換対称群S3の3,200グラフを単純パス全列挙と比較し、距離・群積・頂点重複なし・辺方向を検査。 [実行できる使用例](../../examples/graph_basic/shortest_nonzero_path.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
