---
title: Radix Heap（共通入口）
documentation_of: //ordered_structure/radix_heap.hpp
---

# Radix Heap（共通入口）

既存iwiwi実装を共有します。最後に取り出したkeyとの最高相違bitでbucketを分け、単調に取り出す整数keyを扱います。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/radix_heap.hpp"
int main() {
    cp::RadixHeap<unsigned long long,int> s; s.push(3,10); s.push(8,20); s.pop();
    std::cout << s.top_key() << "\n";
}
```

出力: `8`。

## APIと使用条件

`RadixHeap<Key,Value>`、push(key,value)、top_key/top_value/pop/size/empty/clear。Keyは符号なし32/64bitのみ。

初期の最後のkeyは0。pushするkeyは直前に取り出したkey以上。topもbucketをpullするので、その後のpushにも同じ下限が必要です。同keyのValue順は未規定。元のライセンス・出典は変更していません。

## 計算量

key幅Wに対しpush O(1)、pop/topは償却O(W)、領域O(要素数+W)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#radix_heap) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
