---
title: 木の距離分布
documentation_of: //tree/tree_distance_distribution.hpp
---

# 木の距離分布

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#tree_distance_distribution) / [実装](../../tree/tree_distance_distribution.hpp) / [一次資料と設計](../b_7_11/references.md)

重心ごとの深さ頻度列の自己畳み込みから、同じ枝内の自己畳み込みを引いて全頂点対の距離を数えます。

## usage

```cpp
#include <iostream>
#include "tree/tree_distance_distribution.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    auto c=cp::tree_distance_distribution(g); std::cout << c[1] << ' ' << c[2] << '\n';
}
```

出力:

```text
2 1
```

## API・使用条件

`tree_distance_distribution(g)`→vector<long long>（長さV）。無向森、重み無視、距離は辺数。answer[d]は異なる頂点の非順序対、answer[0]=0。別成分の対は含まない。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

ContourIndexとexact convolution_llを共有します。畳み込み長<=2^24、係数と集計途中のsigned64範囲に従います。ordered対を数えて2で割り、自己対は除きます。孤立頂点のみなら全て0、空なら空。

## 計算量

時間O(V log²(V+1))、領域O(V log(V+1))。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 重心ごとの深さ頻度列の自己畳み込みから、同じ枝内の自己畳み込みを引いて全頂点対の距離を数えます。 [実行できる小問題](../../examples/tree/tree_distance_distribution_b.cpp)。

関連する出題: [Frequency Table of Tree Distance](https://judge.yosupo.jp/problem/frequency_table_of_tree_distance)。入出力仕様の参照先であり、オンラインACの主張ではありません。

木の距離分布を必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
