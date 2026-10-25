---
title: ラベル付き連結グラフ数
documentation_of: //graph_combinatorics/connected_graph_count.hpp
---

# ラベル付き連結グラフ数

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#connected_graph_count) / [実装](../../graph_combinatorics/connected_graph_count.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

n頂点までのラベル付き単純無向連結グラフ数を、任意精度または指定した環で計算します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/connected_graph_count.hpp"
int main() {
    for(const auto& x:cp::connected_graph_counts(4)) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
0 1 1 4 38
```

## API・使用条件

`connected_graph_counts<T=BigInt>(n)`→vector<T>の[0,n]。Tは整数からの構築、+、−、*、複合減算を持つ環型。

c[0]=0,c[1]=1。ループ・多重辺なし、頂点ラベルを区別。n<0はinvalid_argument。BigInt既定は厳密値、固定幅整数を指定する場合は中間値も範囲内であること。合成数modにも対応、除算不要。

## 仕組み

全グラフ数a[n]=2^{n(n−1)/2}から、頂点0を含む成分サイズk<nを選ぶ `C(n−1,k−1)c[k]a[n−k]` を引きます。Pascalの二項係数を共有。

## 計算量

O(n²)回の環演算、O(n²)個の環要素。BigIntのビット長はO(n²)、多倍長演算をO(1)とは数えません。

## 使用する問題・検証

ラベル付きネットワークの連結確率、連結成分による数え上げDPの基本係数。

無向n<=6全グラフと比較。BigIntの既知値、合成数法12でも検証。 [実行できる使用例](../../examples/graph_combinatorics/connected_graph_count.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
