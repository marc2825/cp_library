---
title: Runs Enumeration
documentation_of: //string_algorithm/runs_enumeration.hpp
---

# Runs Enumeration

最小周期pを持ち、長さが2p以上で左右へ延長できない連続区間をすべて列挙します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#runs_enumeration) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`runs_enumeration(string_view)`→`vector<StringRun>{period,left,right}`。unsigned byte、0始まり半開区間、長さ<INT_MAX。

正順・逆順の文字順それぞれで次にsuffix順位が小さい位置を求め、Lyndon根をLCE/LCSで延ばします。空・1byteは空。出力は(period,left,right)の辞書順で重複を除去。UTF-8の文字単位には変換しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/runs_enumeration.hpp"

int main() {
    auto runs = cp::runs_enumeration("ababab");
    std::cout << runs[0].period << " " << runs[0].left << " " << runs[0].right << "\n";
}
```

出力:

```text
2 0 6
```

[コンパイルして動く使用例](../../examples/string_algorithm/runs_enumeration_b.cpp)。

## 計算量

O(N log N)時間・領域。SA倍増法とLCPのSparse Table、候補のソートを含みます。

## 使用する問題・検証

[Run Enumerate](https://judge.yosupo.jp/problem/runenumerate)。

長さ12までの全二値列を、最小周期と左右極大性の直接検査と比較。3万byteの同一文字も確認。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
