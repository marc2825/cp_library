---
title: Dynamic Bitset
documentation_of: //static_query/dynamic_bitset.hpp
---

# Dynamic Bitset

実行時長のbit列をuint64の配列に格納し、word単位の集合演算とshiftを行います。最後のwordの未使用bitを常に0に保ちます。

## 使用例

```cpp
#include <iostream>
#include "static_query/dynamic_bitset.hpp"
int main() {
    cp::DynamicBitset s(130); s.set(63); s.set(64);
    std::cout << (s<<1).find_next(64) << " " << s.count() << "\n";
}
```

出力: `64 2`。

## APIと使用条件

`DynamicBitset(size_t n=0,bool value=false)`、test/set/flip(p)、reset/count/any、&,|,^,~、<<,>>、find_next(p=0)。

二項演算は同じ長さ同士。shift>=Nは全0。find_nextはp以上の最初の1、未発見はsize()。0長、64境界、size_tの大きいshiftも可。

## 計算量

点操作O(1)、bit演算・shift/count O(ceil(N/64))、find_nextは探索word数、領域O(ceil(N/64))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#dynamic_bitset) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
