---
title: Succinct Bit Vector
documentation_of: //static_query/succinct_bit_vector.hpp
---

# Succinct Bit Vector

bit本体を64bit wordに詰め、各word直前までの1の数を保持します。rankから二分探索してselectを得ます。

## 使用例

```cpp
#include <iostream>
#include "static_query/succinct_bit_vector.hpp"
int main() {
    cp::SuccinctBitVector s(5); s.set(1); s.set(4); s.build();
    std::cout << s.rank(4) << " " << s.select(1) << "\n";
}
```

出力: `1 4`。

## APIと使用条件

`SuccinctBitVector(n)`、set(p,bool=true)、get(p)、build()、rank(r,bool=true)、select(k,bool=true)、size。

rankは[0,r)の個数、selectは0始まりのk番目の位置で未発見は-1。set後はbuildを再実行してからrank/select。nは非負int。厳密なN+o(N)bitの理論的succinct構造ではなく、競プロで使うword+prefix形式です。

## 計算量

領域N bit+O(N/64)個の整数、build O(N/64)、rank/get/set O(1)、select O(log(N+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#succinct_bit_vector) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
