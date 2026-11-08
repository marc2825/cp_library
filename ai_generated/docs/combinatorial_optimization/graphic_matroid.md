---
title: Graphic Matroid
documentation_of: //combinatorial_optimization/graphic_matroid.hpp
---

# Graphic Matroid

無向多重グラフの辺集合が森を成すかを、既存DSUで判定する独立性オラクルです。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#graphic_matroid) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`GraphicMatroid(vertices,vector<pair<int,int>> edges)`、`size()`、`independent(elements)` / `operator()(elements)`。

要素IDは辺の入力順。自己辺は独立でなく、同じ辺の重複指定もfalse。平行辺を2本選ぶと閉路。頂点・要素IDは例外で検査。オラクルは呼出しごとにDSUを再構築するため、状態に依存しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/graphic_matroid.hpp"

int main() {
    cp::GraphicMatroid forest(3, {{0, 1}, {1, 2}, {2, 0}});
    std::cout << forest.independent({0, 1}) << " " << forest.independent({0, 1, 2}) << "\n";
}
```

出力:

```text
1 0
```

[コンパイルして動く使用例](../../examples/combinatorial_optimization/graphic_matroid_b.cpp)。

## 計算量

構築O(n+m)、1判定O(n+|elements| α(n))時間、O(n)領域。

## 使用する問題・検証

最大森、色容量付き全域森のMatroid Intersection。

各連結成分の辺数と頂点数から直接判定する全subset比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
