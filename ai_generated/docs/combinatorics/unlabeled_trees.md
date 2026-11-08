---
title: ラベルなし木の同型類数
documentation_of: //combinatorics/unlabeled_trees.hpp
---

# ラベルなし木の同型類数（C-04）

[ヘッダ](../../combinatorics/unlabeled_trees.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#unlabeled_trees)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/unlabeled_trees.hpp"
int main() {
    const auto counts = cp::unlabeled_tree_counts(8);
    for (int i = 1; i <= 8; ++i) std::cout << counts.rooted[i] << (i == 8 ? '\n' : ' ');
    for (int i = 1; i <= 8; ++i) std::cout << counts.unrooted[i] << (i == 8 ? '\n' : ' ');
}
```

出力:

```text
1 1 2 4 9 20 48 115
1 1 1 2 3 6 11 23
```

## API・条件

`unlabeled_tree_counts(n,max_vertices=2000)` は `UnlabeledTreeCounts{rooted,unrooted}` を返します。配列の添字は頂点数0..n。値はBigIntです。兄弟の順序を区別しない非平面木の同型類を数え、根付きと無根を分けます。空木は双方0、1頂点は双方1。

予算超過は `length_error`。Boostのヘッダが必要です。具体的な木の列挙・同型判定は既存の木アルゴリズムを使います。

## 仕組み

根付き木T(x)は `T(x)=x product_{k≥1}(1−x^k)^−t_k`。対数微分して `b_k=sum_{d|k} d*t_d`、`(n−1)t_n=sum_{k=1}^{n−1} b_k*t_(n−k)` を順に計算します。

無根木は頂点根/辺根の関係から `u_n=t_n−(sum_{i=1}^{n−1}t_i*t_(n−i)−[n even]t_(n/2))/2`。除算は整数として厳密です。

## 計算量

O(n²)回の多倍長演算、O(n)個の多倍長整数。ビット計算量とメモリ量は係数の桁数にも依存します。

## 使用する問題

非同型木の個数、根付き構造の母関数、Pólya型の非ラベル組合せクラス。

## 検証

n≤7のPrüfer列から全ラベル木を生成し、根付き括弧表現で同型類を独立集計。n≤10の既知列とn≤60のEuler積の関係も照合。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
