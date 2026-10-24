---
title: Static Range Mode
documentation_of: //static_query/static_range_mode.hpp
---

# Static Range Mode（D-25）

静的列の区間で最も多い値と出現数を返します。ブロック区間ごとの候補を前計算し、問い合わせでは中央の候補と両端の値を出現位置配列で再検証します。同数ならCompare順で最小の値です。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#static_range_mode)

## API・使用条件

`StaticRangeMode<T,Compare>(values,compare={},block_size=0)`。`mode(l,r)` → `optional<pair<T,int>>`、`size()`。0のblock_sizeは自動選択。

値はコピー可能、Compareはstrict weak ordering。比較で同値の値は同じ頻度として数えます。空区間はnullopt。添字はassert。構築後の更新はありません。

## 計算量・保存領域

ブロック幅 B に対し構築 O(n²/B+n log n)、問い合わせ O(B log n)、領域 O(n+(n/B)²)。既定 B=max(1,⌊√(n+1)⌋)。空の列も可。

## 使用例

[コンパイル可能な使用例](../../examples/static_query/static_range_mode.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../static_query/static_range_mode.hpp"
int main(){
    cp::StaticRangeMode<int> mode({2,1,2,3,1,2});
    auto a=*mode.mode(0,6),b=*mode.mode(1,5);
    std::cout<<a.first<<' '<<a.second<<'\n'<<b.first<<' '<<b.second<<'\n';
}
```

```text
2 3
1 2
```

## 使用する問題

Static Range Mode Query。半数超えだけを知りたい場合は既存RangeMajorityの方が軽量です。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Chan et al., Linear-Space Data Structures for Range Mode Query](https://doi.org/10.4230/LIPIcs.STACS.2012.290)。ここでは単純なブロック候補表と出現位置の二分探索を使います。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
