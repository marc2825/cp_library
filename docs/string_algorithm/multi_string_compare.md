---
title: 複数文字列の厳密比較
documentation_of: //string_algorithm/multi_string_compare.hpp
---

# 複数文字列の厳密比較

文字列ごとに異なる番兵を使う連結SAとLCP RMQから、任意のsubstring同士を衝突なしで比較します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#multi_string_compare) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`MultiStringCompare(vector<string>)`、`size()`、`lcp(a,l,r,b,x,y)`、`compare(a,l,r,b,x,y)`→-1/0/1。

byte値0..255に対して番兵は256以上の整数。LCPを各指定区間の長さで制限するため番兵をまたぎません。空文字列・空区間可。N<INT_MAX、文字列数≤INT_MAX-256。id/区間違反は例外。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/multi_string_compare.hpp"

int main() {
    cp::MultiStringCompare strings({"banana", "band"});
    std::cout << strings.lcp(0, 0, 6, 1, 0, 4) << " " << strings.compare(0, 0, 6, 1, 0, 4) << "\n";
}
```

出力:

```text
3 -1
```

[コンパイルして動く使用例](../../examples/string_algorithm/multi_string_compare_b.cpp)。

## 計算量

番兵を含む合計長Nで構築O(N log N)時間・領域、問い合わせO(1)。

## 使用する問題・検証

多数の文字列のsubstringソート、厳密な共通prefixの問い合わせ。

空・NUL・FFを含む複数文字列の全区間対を、unsigned byteの直接比較と照合。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
