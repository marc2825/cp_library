---
title: Hilbert Order Mo
documentation_of: //offline_query/hilbert_order_mo.hpp
---

# Hilbert Order Mo

問い合わせの左右端を平面の点と見なし、Hilbert曲線の順に処理します。近い区間を続けて処理し、現在区間の移動量を抑えます。

## 使用例

```cpp
#include <iostream>
#include "offline_query/hilbert_order_mo.hpp"
int main() {
    cp::HilbertOrderMo m(4); m.add_query(1,3); int total=0,answer=0;
    m.run([&](int i){total+=i;},[&](int i){total-=i;},[&](int){answer=total;});
    std::cout << answer << "\n";
}
```

出力: `3`。

## APIと使用条件

`hilbert_order(uint32 x,y,unsigned bits=31)`、`HilbertOrderMo(n)`、add_query(l,r)、query_order/query_count、run(add,remove,answer)または左右別5callback。

座標は0<=x,y<2^bits、bits<=31。区間は空可。run開始前に利用者の集約状態を空[0,0)へ戻します。answerのIDは登録順。Hilbert順が通常Moより必ず高速になる保証はありません。

## 計算量

順序計算O(Q log N+Q log Q)（実装は31bit固定）、移動は実測依存。一般的な最悪上界はO(NQ)。領域O(Q)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Static Range Count Distinct](https://judge.yosupo.jp/problem/static_range_count_distinct)。各区間でsetを作る解と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#hilbert_order_mo) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
