---
title: 文字列の最小巡回シフト
documentation_of: //string_algorithm/minimum_rotation.hpp
---

# 文字列の最小巡回シフト

辞書順で最小の巡回シフトの開始位置を、二つの候補を進めて求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#minimum_rotation) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`minimum_rotation(string_view)`→int。unsigned byte順、空は0。同じ最小文字列が複数あれば最初の位置。

比較で負のchar順を使いません。結果がiならs[i..N)+s[0..i)が最小です。入力を変更しません。N≤INT_MAX。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/minimum_rotation.hpp"

int main() {
    std::cout << cp::minimum_rotation("baca") << "\n";
}
```

出力:

```text
3
```

[コンパイルして動く使用例](../../examples/string_algorithm/minimum_rotation_b.cpp)。

## 計算量

O(N)時間・O(1)追加領域。文字列を2回連結して保存する必要はありません。

## 使用する問題・検証

ネックレスの正規化、巡回同値な文字列の代表化。

長さ12までの全二値列を、全巡回シフトの直接比較と照合。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
