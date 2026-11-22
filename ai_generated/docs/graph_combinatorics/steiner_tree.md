---
title: Steiner Tree DP
documentation_of: //graph_combinatorics/steiner_tree.hpp
---

# Steiner Tree DP

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#steiner_tree) / [実装](../../graph_combinatorics/steiner_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

terminal集合を部分集合ごとに併合し、multi-source Dijkstraで根の位置を広げるDreyfus–Wagner法です。使用した辺も復元します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/steiner_tree.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    std::cout << cp::steiner_tree(g,{0,2})->cost << '\n';
}
```

出力:

```text
3
```

## API・使用条件

`steiner_tree(g,vector<int> terminals)`→optional<SteinerTreeResult<T>>（cost,vertices,edge_ids）。非負無向、重複terminalは除去。異なるterminal数K<=20。接続不能はnullopt、terminal空は費用0で空の木。

edge_ids は g.edges() の添字、vertices は昇順。有向・負の辺は`invalid_argument`、範囲外の terminal は`out_of_range`、異なる terminal が21個以上は`length_error`です。

## 仕組みと境界

復元stateを重複処理せず、得られた辺集合から閉路と非terminal葉を除きます。ゼロ重みも扱います。距離・併合費用の中間値がTの有限範囲に収まる必要があります。Kの上限とは別に状態数のメモリ・時間を見積もってください。

## 計算量

時間O(3^K V+2^K(V+E) log(V+E+1))、領域O(2^K V+V+E)。 Vは頂点数、Eは論理辺数。T の加算・比較を O(1) として数えています。本環境で V=100・E=1000 なら K=12 は約0.1秒、K=16 は約4秒・約130MB。K=20 は 2^20·V 個の状態（V=100 で約2GB）を確保します。

## 使用する問題・検証

想定問題: 指定した K 頂点を全て連結にする最小重みの辺集合。[Library Checker: Minimum Steiner Tree](https://judge.yosupo.jp/problem/minimum_steiner_tree)（N≤100、M≤1000、K≤12）は cost と edge_ids を出力すれば解けます（提出は未確認）。 [実行できる小問題](../../examples/graph_combinatorics/steiner_tree_b.cpp)。

検証では、ループ・平行辺・重み0を含む7頂点以下・10辺以下のランダムグラフ550個と、重複を含む4個以下の terminal で全辺部分集合の最小費用と比較し、edge_ids が重複のない森で terminal を連結し、費用の和が一致することを確かめます。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/combinatorics_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
