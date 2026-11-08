---
title: Wildcard Pattern Matching
documentation_of: //string_algorithm/wildcard_pattern_matching.hpp
---

# Wildcard Pattern Matching

任意の1byteに一致するワイルドカードを含むパターンの全一致位置を、正確な整数畳み込みで求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#wildcard_pattern_matching) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`wildcard_pattern_matching(text,pattern,unsigned char wildcard='?')`→vector<int>。ワイルドカードはtext・patternの両方で有効。

固定長の1byte wildcardです。可変長のglob記号*は扱いません。byte+1を数値、wildcardを0としてΣab(a-b)²=0を検査。ハッシュ衝突はありません。空patternは全境界、patternが長ければ空。非空の結果長≤2^24が必要。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/wildcard_pattern_matching.hpp"

int main() {
    auto positions = cp::wildcard_pattern_matching("abacaba", "a?a");
    std::cout << positions[0] << " " << positions[1] << " " << positions[2] << "\n";
}
```

出力:

```text
0 2 4
```

[コンパイルして動く使用例](../../examples/string_algorithm/wildcard_pattern_matching_b.cpp)。

## 計算量

L=切上げ2冪(|text|+|pattern|-1)としてO(L log L)時間・O(L)領域。3本の整数畳み込みはそれぞれ3法NTTを使用。

## 使用する問題・検証

[Wildcard Pattern Matching](https://judge.yosupo.jp/problem/wildcard_pattern_matching)。

両側のwildcard・任意byte・空patternを直接全照合と比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
