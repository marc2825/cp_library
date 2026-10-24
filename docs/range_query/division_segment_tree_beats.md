---
title: Floor Division Segment Tree Beats
documentation_of: //range_query/division_segment_tree_beats.hpp
---

# Floor Division Segment Tree Beats（D-24）

要仕様化だったD-24を「非負整数列への区間floor除算・加算・代入」と定義しました。区間min/maxの商が同じなら代入へ、減少量が同じなら一様加算へまとめ、どちらも成立しない節だけ子へ降ります。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#division_segment_tree_beats)

## API・使用条件

`DivisionSegmentTreeBeats(vector<long long>)`。`range_add(l,r,x)`, `range_assign(l,r,x)`, `range_divide(l,r,d)`, `range_sum/min/max(l,r)`, `get(p)`, `size()`。空区間の和・最大は0、最小はLLONG_MAX。

初期値・代入・外部からの加算は非負、除数は1以上（assert）。区間和・遅延値を含む全中間値はlong longに収まる必要があります。負数のfloor除算には対応しません。

## 計算量・保存領域

構築 O(n)、加算・代入・和・min/maxは O(log n)、floor除算は最悪 O(n)。領域 O(n)。min/maxによる枝刈りを行いますが、任意の更新列に対する O(log n) の保証はしません。

## 使用例

[コンパイル可能な使用例](../../examples/range_query/division_segment_tree_beats.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../range_query/division_segment_tree_beats.hpp"
int main(){
    cp::DivisionSegmentTreeBeats tree({10,21,32,43});
    tree.range_divide(0,4,3); tree.range_add(1,3,2);
    std::cout<<tree.range_sum(0,4)<<' '<<tree.range_min(0,4)<<' '<<tree.range_max(0,4)<<'\n';
}
```

```text
38 3 14
```

## 使用する問題

区間除算・代入・加算と和の問い合わせ。大きな値が除算で縮む更新列に向きます。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[jiry_2, Segment Tree Beats](https://codeforces.com/blog/entry/57319)。失敗した節だけ降りる考え方をmin/maxの商と変化量に適用しました。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
