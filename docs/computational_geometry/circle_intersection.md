---
title: Circle-Line / Circle-Circle Intersection
documentation_of: //computational_geometry/circle_intersection.hpp
---

# Circle-Line / Circle-Circle Intersection

円と直線、円同士の交点を求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#circle_intersection) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::floating::Circle(center={},radius=0)`、`circle_line_intersection(circle,line,tol)`、`circle_circle_intersection(a,b,tol)`→`{coincident,points}`。

半径は有限・非負。交点なしは空、接する場合は1点、交差は2点。同一の正半径円はcoincident=trueかつ空。零半径の同一点は1交点。Toleranceは中心距離・半径の比較と高さ二乗の判定へ適用する絶対値です。尺度を調整し、非有限途中値や数値的に潰れた直線は例外です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/circle_intersection.hpp"

int main() {
    namespace g = cp::geometry::floating;
    auto result = g::circle_circle_intersection(g::Circle({0, 0}, 1), g::Circle({2, 0}, 1));
    std::cout << result.points.size() << " " << double(result.points[0].x) << "\n";
}
```

出力:

```text
1 1
```

[コンパイルして動く使用例](../../examples/computational_geometry/circle_intersection_b.cpp)。

## 計算量

O(1)時間・領域。有限交点は座標の辞書順です。

## 使用する問題・検証

[AOJ CGL_7_D](https://onlinejudge.u-aizu.ac.jp/problems/CGL_7_D)、[CGL_7_E](https://onlinejudge.u-aizu.ac.jp/problems/CGL_7_E)。

全交点の円・直線への所属、接点・同一円・零半径・非交差を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
