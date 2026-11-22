---
title: C4の個数
documentation_of: //graph_combinatorics/count_four_cycles.hpp
---

# C4の個数

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#count_four_cycles) / [実装](../../graph_combinatorics/count_four_cycles.hpp) / [一次資料と設計](../b_7_11/references.md)

次数と番号の順序で一番大きい頂点を固定し、その下の2辺walkを同じ終点ごとに組み合わせます。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/count_four_cycles.hpp"
int main() {
    cp::Graph<int> g(4); for(int u=0;u<4;++u) for(int v=u+1;v<4;++v) g.add_edge(u,v);
    std::cout << cp::count_four_cycles(g) << '\n';
}
```

出力:

```text
3
```

## API・使用条件

`count_four_cycles(g)`→uint64_t。単純無向、ループ・平行辺はinvalid_argument。重み無視。

## 仕組みと境界

4頂点集合そのものではなく、異なる長さ4の閉路を数えます。K4には3個。対角線があっても数えます。結果がuint64を超える場合はoverflow_error。

## 計算量

時間O(V+E sqrt(E+1))（前処理sortを含む）、領域O(V+E)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: 単純無向グラフの長さ4の閉路の総数（二部グラフでは K2,2 部分グラフの数）。[Library Checker: Counting C4](https://judge.yosupo.jp/problem/counting_c4) は辺ごとの個数かつ多重辺ありのため、この API だけでは解けません。 [実行できる小問題](../../examples/graph_combinatorics/count_four_cycles_b.cpp)。

検証では、8頂点未満のランダム単純グラフ280個で、異なる4頂点の順序付き閉路の全列挙数の 1/8 と比較します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/combinatorics_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
