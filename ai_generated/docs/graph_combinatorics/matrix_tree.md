---
title: 全域木の個数（行列木定理）
documentation_of: //graph_combinatorics/matrix_tree.hpp
---

# 全域木の個数（行列木定理）

無向多重グラフの全域木の個数を、素数 mod で数えます。重み付きでは全域木ごとの辺重みの積の総和を求めます。

[ラプラシアン・余因子・消去の図解](../../visualizations/graph_combinatorics/index.html#trees)

## 原理

辺 u—v の重み w を L[u,u] と L[v,v] に加え、L[u,v] と L[v,u] から引きます。通常の個数では全て w=1。一頂点の行と同じ頂点の列を除いた余因子の行列式が答えです。どの頂点の行・列を除いても値は一致します。元のグラフからその頂点を削除するわけではありません。

多重辺は各辺の寄与を別々に加えます。自己ループは全域木には入らないので、ラプラシアンへの寄与も除外します。

## usage

```cpp
#include <cassert>
#include "graph_combinatorics/matrix_tree.hpp"
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(0,2,5);
    assert(cp::count_spanning_trees(g)==3); // cost を無視
    assert(cp::count_weighted_spanning_trees<101>(g)==31); // 2*3+2*5+3*5
    assert(cp::count_weighted_spanning_trees<101>(g,0)==31);
    assert(cp::count_spanning_trees(cp::Graph<>(1))==1);
    assert(cp::count_spanning_trees(cp::Graph<>(0))==0);
}
```

## API・条件

- `int count_spanning_trees<Mod=998244353>(g,removed_vertex=-1)`：重みを無視して個数を返します。T の重み演算は不要です。
- `int count_weighted_spanning_trees<Mod=998244353>(g,removed_vertex=-1)`：辺 cost を重みとして、積の和を返します。T は bool 以外の整数型、符号付き・符号なしの64ビットまでを想定します。負・0の重みも剰余として扱います。

無向であれば非連結・多重辺・自己ループが使えます。Mod は int で表せる素数で、コンパイル時に確認します。合成数では逆元による消去はできず、この入口には渡せません。

removed_vertex=-1 は末尾の頂点の行・列を除きます。それ以外は [0,N) の範囲。N=0 では -1 を指定し、答えはこのライブラリの規約として0。N=1 は0次の行列式・空積が1なので、ループの有無に関係なく1です。

結果は [0,Mod) の int。非連結なら0ですが、連結でも真の個数が Mod の倍数なら0です。重み付きの相殺でも0になり得ます。したがって、この結果を連結判定に使わないでください。

## 計算量・整数範囲

余因子を直接構築するため時間 O(N³+M+N log Mod)、作業領域 O(N²)。Mod を固定と見れば通常 O(N³+M)。入力辺は一度だけ走査し、Graph は変更しません。

各値を Mod の範囲に正規化し、乗算は long long で行います。Mod≤INT_MAX なので剰余同士の積は signed 64ビットに収まります。真の全域木数が long long を超えても、剰余は求められます。浮動小数点による丸めは使いません。

## 問題・検証

[Library Checker Counting Spanning Tree (Undirected)](https://judge.yosupo.jp/problem/counting_spanning_tree_undirected) の [ラッパー](../../verify/graph_combinatorics/counting_spanning_tree_undirected.test.cpp)。[動く使用例](../../examples/graph_combinatorics/matrix_tree.cpp)。

全ての N−1 本の辺部分集合を実際に選んで全域木を確認し、個数と重みの積の和を比較します。全ての除去行・列、負の重み、整数極値、多重辺・ループ、素数2、完全グラフの Cayley の公式も検証します。[行列式](determinant_mod.md) / [カテゴリの検証](README.md#動く例問題検証) / [参考資料](references.md)
