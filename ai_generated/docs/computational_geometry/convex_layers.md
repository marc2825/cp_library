---
title: Convex Layers
documentation_of: //computational_geometry/convex_layers.hpp
---

# Convex Layers

凸包の全境界点を1層として取り除く操作を繰り返し、元の各点の層番号を返します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#convex_layers) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::exact::convex_layers(points)`→vector<int>。最外層0、重複座標は同じ層。

境界上の共線点も同じ層で除きます。全共線・同一点群は1層です。既存の整数凸包を共有し、128bit中間値の条件と超過例外を引き継ぎます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/convex_layers.hpp"

int main() {
    auto layer = cp::geometry::exact::convex_layers({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {2, 2}});
    std::cout << layer[0] << " " << layer[4] << "\n";
}
```

出力:

```text
0 1
```

[コンパイルして動く使用例](../../examples/computational_geometry/convex_layers_b.cpp)。

## 計算量

最大N層としてO(N² log N)時間、O(N)追加領域。高度な高速onion分解ではありません。

## 使用する問題・検証

点群のonion depth、凸包を順に取り除く数え上げ。

すべての支持直線から境界を直接判定する独立な剥離と、重複・共線を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
