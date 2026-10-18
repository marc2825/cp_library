---
title: Manhattan MST（共通入口）
documentation_of: //computational_geometry/manhattan_mst.hpp
---

# Manhattan MST（共通入口）

3.7の4方向sweepとKruskalによるManhattan距離MSTを幾何から共有します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#manhattan_mst) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`cp::manhattan_mst(vector<pair<long long,long long>>)`→MinimumSpanningForestResult<long long>。

[既存のAPI・条件](../graph_basic/manhattan_mst.md)と同じ実体。空・重複点可。128bitで変換し、辺重みと総費用はlong longに収まる必要があります。出力辺のidxは候補グラフに対するIDです。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../computational_geometry/manhattan_mst.hpp"

int main() {
    auto result = cp::manhattan_mst({{0, 0}, {1, 0}, {1, 2}});
    std::cout << result.cost << "\n";
}
```

出力:

```text
3
```

[コンパイルして動く使用例](../../examples/computational_geometry/manhattan_mst_b.cpp)。

## 計算量

O(N log N)時間、O(N)領域。

## 使用する問題・検証

[Manhattan MST](https://judge.yosupo.jp/problem/manhattanmst)。

全点対の距離を使う密なPrimとの比較。既存3.7 Bの検証も共有。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
