---
title: Matrix Tree Theorem（共通入口）
documentation_of: //linear_algebra/matrix_tree.hpp
---

# Matrix Tree Theorem（共通入口）

3.10の行列木定理を共有し、無向多重グラフの全域木を数えます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#matrix_tree) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`count_spanning_trees<Mod>(Graph<T>,removed=-1)`、`count_weighted_spanning_trees<Mod>(...)`。無向グラフ、素数Mod。

重み付きは各全域木の辺重みの積の総和。自己辺は無視し、多重辺は別の選択肢。空グラフ0、1頂点1。[既存条件と説明](../graph_combinatorics/matrix_tree.md)を共有し、誤った有向入力やremovedは既存assert契約です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_algebra/matrix_tree.hpp"

int main() {
    cp::Graph<int> g(3);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 3); g.add_edge(0, 2, 5);
    std::cout << cp::count_weighted_spanning_trees(g) << "\n";
}
```

出力:

```text
31
```

[コンパイルして動く例](../../examples/linear_algebra/matrix_tree_b.cpp)。

## 計算量

n頂点・m辺でO(n³+m)時間、O(n²)領域。

## 使用する問題・検証範囲

[Counting Spanning Trees (Undirected)](https://judge.yosupo.jp/problem/counting_spanning_tree_undirected)。

既存の辺subset全列挙・行列木検証と今回の共通入口を共有。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
