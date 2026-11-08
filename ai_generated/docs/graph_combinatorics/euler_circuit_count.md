---
title: Euler閉路数（BEST定理）
documentation_of: //graph_combinatorics/euler_circuit_count.hpp
---

# Euler閉路数（BEST定理）

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#euler_circuit_count) / [実装](../../graph_combinatorics/euler_circuit_count.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

有向多重グラフの全辺を1回ずつ使い、指定始点へ戻る元辺ID列の数を素数法で返します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/euler_circuit_count.hpp"
int main() {
    cp::Graph<int> g(2,cp::GraphKind::Directed);
    for(int i=0;i<2;++i) { g.add_edge(0,1); g.add_edge(1,0); }
    std::cout << cp::count_euler_circuits(g,0) << ' ' << cp::count_euler_circuits(g,0,0) << '\n';
}
```

出力:

```text
4 2
```

## API・使用条件

`count_euler_circuits<Mod=998244353>(g,root,first_edge=-1)`→int。first_edgeを指定すると先頭辺を固定。Modはint範囲の素数（コンパイル時検査）。

辺IDを区別するので平行辺も別の選択です。回転同値ではなく始点付きの列を数えます。ループ可、重みは無視。孤立点を除いた有効部分でMatrix Treeを計算。辺なし・有効なrootなら空列1個。次数不均衡/非連結/辺ありでroot孤立は0。不正root/first_edge/無向は例外。

## 仕組み

out-Laplacianからrootの行列式minorでrootへ向かう有向全域木数tを求めます。固定先頭辺の数は `t Π_v(outdeg(v)−1)!`、先頭自由ならさらにoutdeg(root)倍。ループは次数に寄与しLaplacianからは相殺されます。

## 計算量

O(n³+m)時間、O(n²+m)領域（実際の行列は非孤立頂点だけ）。

## 使用する問題・検証

有向Euler tourの数え上げ、文字列の隣接関係からの復元数、ラベル付きde Bruijn型の巡回。

800小多重グラフの全辺順列をDFS列挙。先頭固定・自由、法2、孤立点、8自己ループの8!/7!を検査。 [実行できる使用例](../../examples/graph_combinatorics/euler_circuit_count.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
