---
title: 彩色多項式
documentation_of: //graph_combinatorics/chromatic_polynomial.hpp
---

# 彩色多項式

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#chromatic_polynomial) / [実装](../../graph_combinatorics/chromatic_polynomial.hpp) / [一次資料と設計](../b_7_11/references.md)

独立集合による頂点集合の分割数をsubset DPで数え、下降階乗(x)_kの基底から通常の係数へ変換します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/chromatic_polynomial.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto p=cp::chromatic_polynomial<long long>(g);
    for(std::size_t i=0;i<p.size();++i) { if(i) std::cout << ' '; std::cout << p[i]; }
    std::cout << '\n';
}
```

出力:

```text
0 2 -3 1
```

## API・使用条件

`chromatic_polynomial<Ring>(g)`→vector<Ring>（昇冪、長さV+1）。無向、V<=20、空なら{1}。平行辺は一本と同じ、ループがあれば零多項式。Ringは0,1,整数からの構築と+,-,*が必要。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

各blockに残集合の最下位bitを必ず含め、blockの順列による多重計数を防ぎます。係数型の溢れは利用者が避けます。V=20でも計算は重く、上限は実行時間の保証ではありません。

## 計算量

時間O(V 3^V+V²)、領域O(V 2^V)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 独立集合による頂点集合の分割数をsubset DPで数え、下降階乗(x)_kの基底から通常の係数へ変換します。 [実行できる小問題](../../examples/graph_combinatorics/chromatic_polynomial_b.cpp)。

彩色多項式を必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
