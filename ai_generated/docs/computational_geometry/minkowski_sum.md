---
title: Minkowski Sum
documentation_of: //computational_geometry/minkowski_sum.hpp
---

# Minkowski Sum

二つの入力点群の凸包のMinkowski和を、辺方向のmergeで計算します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#minkowski_sum) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::exact::minkowski_sum(a,b)`→strict CCW凸包。出力の始点は最小(x,y)。

空を含むと空。一点・線分も可。入力の点群から凸包を求めるため、非凸な多角形そのもののMinkowski和ではありません。Point加減の結果はint64、外積は検査付き128bit。辺ベクトルの差もint64内で必要です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/minkowski_sum.hpp"

int main() {
    namespace g = cp::geometry::exact;
    auto hull = g::minkowski_sum({{0, 0}, {1, 0}, {0, 1}}, {{2, 3}});
    std::cout << hull[0].x << " " << hull[0].y << " " << hull.size() << "\n";
}
```

出力:

```text
2 3 3
```

[コンパイルして動く使用例](../../examples/computational_geometry/minkowski_sum_b.cpp)。

## 計算量

入力長N,MでO(N log N+M log M)時間、O(N+M)領域。凸包正規化と結果の整形を含みます。

## 使用する問題・検証

凸領域の移動可能範囲、配置空間、凸点群の和の凸包。

全点対の和から直接作る凸包と、ランダム・退化点群を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
