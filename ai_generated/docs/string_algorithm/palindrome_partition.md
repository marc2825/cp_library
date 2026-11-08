---
title: 回文分割DP
documentation_of: //string_algorithm/palindrome_partition.hpp
---

# 回文分割DP

全prefixの最小回文分割数と、全文について実際の回文区間を復元します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#palindrome_partition) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`palindrome_partition(text)`→`{prefix_minimum,intervals}`。prefix_minimumはN+1項、先頭0。intervalsは半開区間。

suffix linkで同じ差分を持つ回文をまとめます。最小値の起点も保持して分割を復元。空はprefix_minimum={0}と空の分割。N≤INT_MAX-2。最適分割の辞書順最小は保証しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/palindrome_partition.hpp"

int main() {
    auto result = cp::palindrome_partition("abacdc");
    std::cout << result.prefix_minimum.back() << " " << result.intervals.size() << "\n";
}
```

出力:

```text
2 2
```

[コンパイルして動く使用例](../../examples/string_algorithm/palindrome_partition_b.cpp)。

## 計算量

Eertreeのseries linksによりO(N log N)時間、O(N)領域。byte遷移のmap費用はlog 256。

## 使用する問題・検証

最小回文分割と全prefixの最小分割数の計算。

全二値列のO(N²) DPと比較し、全復元区間の回文性と隣接を検査。10万byteの同一文字も確認。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
