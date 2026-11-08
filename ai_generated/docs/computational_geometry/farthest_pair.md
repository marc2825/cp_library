---
title: Farthest Pair
documentation_of: //computational_geometry/farthest_pair.hpp
---

# Farthest Pair

点群の凸包を求め、既存のRotating Calipersで最遠点対を返します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#farthest_pair) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::exact::farthest_pair(points)` / `floating::farthest_pair(points)`→optional<PairResult>。元の入力添字とsquared_distance。

空はnullopt、一点や同一点群は距離0。重複座標は最初の元添字を選びます。同じ最大距離の複数の点対の辞書順最小は保証しません。浮動版の凸包・距離にはEPSを入れません。既存[直径API](rotating_calipers.md)を共有。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/farthest_pair.hpp"

int main() {
    auto pair = cp::geometry::exact::farthest_pair({{0, 0}, {3, 4}, {1, 1}});
    std::cout << static_cast<long long>(pair->squared_distance) << "\n";
}
```

出力:

```text
25
```

[コンパイルして動く使用例](../../examples/computational_geometry/farthest_pair_b.cpp)。

## 計算量

O(N log N)時間、O(N)領域。

## 使用する問題・検証

[AOJ CGL_4_B](https://onlinejudge.u-aizu.ac.jp/problems/CGL_4_B)。

全点対の二乗距離と、元添字・空・一点を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
