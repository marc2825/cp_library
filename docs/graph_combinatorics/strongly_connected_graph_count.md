---
title: 強連結有向グラフ数
documentation_of: //graph_combinatorics/strongly_connected_graph_count.hpp
---

# 強連結有向グラフ数

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#strongly_connected_graph_count) / [実装](../../graph_combinatorics/strongly_connected_graph_count.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

ループのないラベル付き単純有向グラフのうち、強連結なものを数えます。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/strongly_connected_graph_count.hpp"
int main() {
    for(const auto& x:cp::strongly_connected_graph_counts(4)) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
0 1 1 18 1606
```

## API・使用条件

`strongly_connected_graph_counts<T=BigInt>(n)`→vector<T>の[0,n]。Tは加減乗算と複合加減算を持つ環型。

s[0]=0,s[1]=1。逆向き2辺は同時に存在でき、独立に選びます。tournamentの数ではありません。負nはinvalid_argument。BigIntは厳密、合成数mod可。固定幅Tの中間値超過は利用者が避ける必要があります。

## 仕組み

Wrightの漸化式: `eta[n]=2^{n(n−1)}−Σ C(n,k)2^{(n−1)(n−k)}eta[k]`、`s[n]=eta[n]+Σ C(n−1,k−1)s[k]eta[n−k]`（和は1<=k<n）。係数はPascalと2の累乗で作り、階乗除算を使いません。

## 計算量

O(n²)回の環演算、O(n²)個の環要素。BigIntのビット長はO(n²)。

## 使用する問題・検証

相互到達可能なラベル付きネットワークの数、ランダム有向グラフの強連結確率。

有向n<=4の全グラフ、n=5の565080、合成数法12とBigIntの一致を検査。 [実行できる使用例](../../examples/graph_combinatorics/strongly_connected_graph_count.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
