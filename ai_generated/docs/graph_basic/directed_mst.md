---
title: Directed MST（Chu–Liu/Edmonds）
documentation_of: //graph_basic/directed_mst.hpp
---

# Directed MST（Chu–Liu/Edmonds）

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#directed_mst) / [実装](../../graph_basic/directed_mst.hpp) / [一次資料と設計](../b_7_11/references.md)

各頂点の最小入辺を選び、できた有向閉路を縮約します。縮約段の対応を保存し、指定根から全頂点へ届く最小費用の有向木を元の辺IDへ戻します。

## usage

```cpp
#include <iostream>
#include "graph_basic/directed_mst.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,5); g.add_edge(0,2,7); g.add_edge(1,2,1); g.add_edge(2,1,1);
    std::cout << cp::directed_mst(g)->cost << '\n';
}
```

出力:

```text
6
```

## API・使用条件

`directed_mst(g,root=0)` → `optional<DirectedMSTResult<T>>`。有向グラフ（無向グラフは`invalid_argument`）、負辺・多重辺可。到達不能ならnullopt。`cost,parent,parent_edge`を返し、根はparent[root]=root、parent_edge[root]=-1。空グラフは空の結果。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

自己ループと根への入辺は選びません。減算後の辺重みと総費用もTに収まる必要があります。算術型の溢れはoverflow_error。高速なheap版とは異なり、各段で全辺を走査します。

## 計算量

時間O(VE)、全縮約層を保存するため領域O(VE)。 Vは頂点数、Eは論理辺数。重みの比較・加減算はO(1)として数えています。

## 使用する問題・検証

想定問題: 指定した根から全頂点へ届く、最小費用の有向全域木（有向グラフの最小全域有向木）。 [実行できる小問題](../../examples/graph_basic/directed_mst_b.cpp)。

関連する出題: [Directed MST](https://judge.yosupo.jp/problem/directedmst)。入出力仕様の参照先であり、オンラインACの主張ではありません。

検証では、n≤6・m≤10 の有向450グラフを n−1 辺の部分集合の全列挙と比較し、費用と、parent/parent_edge が根から全頂点へ届く木になっていることを検査します。ジャッジ形式の24ケースも独立した Python 解と比較します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/graph_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
