---
title: Minimum Enclosing Circle
documentation_of: //computational_geometry/minimum_enclosing_circle.hpp
---

# Minimum Enclosing Circle

全入力点を含む最小円をランダム増分法で求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#minimum_enclosing_circle) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::floating::minimum_enclosing_circle(points,seed=20261018,Tolerance={})`→optional<Circle>。Circleはcenter,radius。

空はnullopt、一点は半径0。外側の点を見つけたら1/2/3点を境界に持つ円へ更新。全共線の3点は最長直径を選びます。long doubleの全途中値が有限であることが条件。Toleranceは距離二乗と半径二乗の差の絶対許容誤差です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/minimum_enclosing_circle.hpp"

int main() {
    namespace g = cp::geometry::floating;
    auto circle = g::minimum_enclosing_circle({{0, 0}, {2, 0}, {0, 2}});
    std::cout << double(circle->center.x) << " " << double(circle->center.y) << "\n";
}
```

出力:

```text
1 1
```

[コンパイルして動く使用例](../../examples/computational_geometry/minimum_enclosing_circle_b.cpp)。

## 計算量

期待O(N)、最悪O(N³)時間、入力コピーO(N)領域。固定seedで同じ環境では再現できます。

## 使用する問題・検証

最小円による点群の包含、施設・通信範囲の最小半径。

全1点・2点・3点の候補円を独立に列挙し、全点包含と最小半径を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
