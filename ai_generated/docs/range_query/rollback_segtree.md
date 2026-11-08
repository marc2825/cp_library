---
title: Rollback Segtree
documentation_of: //range_query/rollback_segtree.hpp
---

# Rollback Segtree

点代入で変更する全節点の古い値をstackへ保存し、過去の状態へ巻き戻します。検索は履歴を増やしません。

## 使用例

```cpp
#include <iostream>
#include "range_query/rollback_segtree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::RollbackSegtree<long long,op,e> s({1,2,3}); auto token=s.snapshot(); s.set(1,8); s.rollback(token);
    std::cout << s.all_prod() << "\n";
}
```

出力: `6`。

## APIと使用条件

`RollbackSegtree<S,op,e>(vector)`、snapshot()は履歴長、rollback(snapshot)、set/get/prod/all_prod/size。

snapshotは同じ履歴の祖先だけで有効。巻き戻した後に別の更新をした場合、破棄された未来のsnapshotは使えません。非可換可。

## 計算量

構築O(N)、set/prod O(log N)、rollback O(巻き戻す変更節点数)、領域O(N+未巻戻し更新数 log N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#rollback_segtree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
