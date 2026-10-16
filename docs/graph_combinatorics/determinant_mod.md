---
title: 素数 mod の行列式
documentation_of: //graph_combinatorics/determinant_mod.hpp
---

# 素数 mod の行列式

行列木定理の補助として、整数正方行列の行列式を素数 mod で求めます。独立したヘッダとして利用できます。

## usage

```cpp
#include <cassert>
#include "graph_combinatorics/determinant_mod.hpp"
int main() {
    assert(cp::determinant_mod<101>({{0,1},{1,0}})==100); // 行交換の符号
    assert(cp::determinant_mod<101>({{1,2},{2,4}})==0);
    assert(cp::determinant_mod<101>({})==1);
}
```

## API・原理・計算量

`int determinant_mod<Mod=998244353>(std::vector<std::vector<long long>> matrix)`。入力を値で受け取り、剰余を正規化して消去します。呼出し側の行列は変更しません。Mod は int の素数であることをコンパイル時に確認し、戻り値は [0,Mod) の int。

行列は正方であることが条件（assert）。各要素は long long の範囲でよく、負の値も可能です。vector<vector<int>> は直接変換されないため、long long の行列へコピーしてください。

各列で非零 pivot を探し、必要なら行を交換して符号を反転します。pivot の逆元で他行の倍数を求めて下側を消し、pivot の積を返します。非零 pivot がなければ0。0×0の行列式は1です。逆元は Fermat の小定理により pow(pivot,Mod−2) で求めます。

時間 O(N³+N log Mod)、コピーを含む領域 O(N²)。Mod≤INT_MAX のため、正規化した値同士の積と消去時の差は long long に収まります。合成数 mod や実数の行列式を扱う入口ではありません。

## 利用・検証

[行列木定理の図解](../../visualizations/graph_combinatorics/index.html#trees) で消去過程を確認できます。[Library Checker Counting Spanning Tree (Undirected)](https://judge.yosupo.jp/problem/counting_spanning_tree_undirected) がこの補助関数の利用先です。

全順列による符号付き積の和と比較し、行交換、特異行列、負の値、空行列、素数2・101・2147483647を検査しています。[カテゴリ索引](README.md) / [参考資料](references.md)
