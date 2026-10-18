---
title: 半平面交差（明示的な境界矩形内）
documentation_of: //computational_geometry/half_plane_intersection.hpp
---

# 半平面交差（明示的な境界矩形内）

各有向直線の左側の共通部分を、指定した有限の閉矩形内で求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#half_plane_intersection) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::exact::half_plane_intersection(lines,lower,upper)`→vector<RationalPoint>。`floating`版は末尾にTolerance。lower/upperは矩形の最小/最大座標。

境界矩形は必須で、人工的な巨大座標で無限平面を代用しません。無限領域の分類は対象外。結果は空・点・線分にもなり、境界込み。exact版は有理数を保持し既存凸切断を共有。128bitの約分前演算の超過条件を引き継ぎます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/half_plane_intersection.hpp"

int main() {
    namespace g = cp::geometry::exact;
    auto polygon = g::half_plane_intersection({g::Line({0, 0}, {1, 0})}, {-1, -1}, {1, 1});
    std::cout << double(g::polygon_area(polygon).approximate()) << "\n";
}
```

出力:

```text
2
```

[コンパイルして動く使用例](../../examples/computational_geometry/half_plane_intersection_b.cpp)。

## 計算量

半平面H個、初期矩形4点としてO(H²)時間、O(H)領域。逐次凸切断方式。

## 使用する問題・検証

有限領域内での線形不等式の実行可能領域、凸多角形のkernel計算。

全境界直線対の交点を列挙し、全不等式への所属から独立に求めた領域と面積を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
