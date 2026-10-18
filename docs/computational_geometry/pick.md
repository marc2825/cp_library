---
title: 格子点数（Pickの定理）
documentation_of: //computational_geometry/pick.hpp
---

# 格子点数（Pickの定理）

単純な整数多角形について、辺上と内部の格子点数を求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#pick) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`geometry::exact::lattice_polygon_count(polygon)`→`{interior,boundary}`。両方Wide（signed128）。

非自己交差・穴なし・非零面積が前提で、単純性の全検査は行いません。向きは任意、閉点の再掲可。boundary=Σgcd(|dx|,|dy|)、interior=(|area2|−boundary+2)/2。128bitを超える途中演算は例外。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/pick.hpp"

int main() {
    auto count = cp::geometry::exact::lattice_polygon_count({{0, 0}, {3, 0}, {3, 2}, {0, 2}});
    std::cout << static_cast<long long>(count.interior) << " " << static_cast<long long>(count.boundary) << "\n";
}
```

出力:

```text
2 10
```

[コンパイルして動く使用例](../../examples/computational_geometry/pick_b.cpp)。

## 計算量

O(N log C)時間、O(N)入力コピー。Cは座標差の大きさでgcdの費用です。

## 使用する問題・検証

整数多角形の内点計数、境界のgcdを使う数え上げ。

小さい座標の全格子点を直接inside/boundaryへ分類して比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
