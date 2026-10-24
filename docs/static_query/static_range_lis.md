---
title: Static Range LIS
documentation_of: //static_query/static_range_lis.hpp
---

# Static Range LIS（D-26）

静的列の任意の半開区間について、狭義単調増加部分列の最大長を返します。Seaweedの分割統治とunit-Monge積で関係を部分置換へ圧縮し、既存WaveletMatrixの範囲頻度で答えます。同値要素には位置の逆順にrankを付け、同値をLISへ重複採用しません。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#static_range_lis)

## API・使用条件

`StaticRangeLIS<T,Compare>(values={},compare={})`。`query(l,r)` → 長さ、`size()`。空区間は0。Compareをgreaterにすると狭義減少列を扱えます。

値比較はstrict weak ordering。添字はassert。2^29を超える長さはlength_errorですが、実際の上限はメモリに依存します。非減少LISや部分列そのものの復元はこのAPIの対象外です。

## 計算量・保存領域

構築 O(n log² n)、問い合わせ O(log n)、領域 O(n log n) の上界。次の2冪までpaddingします。値比較を O(1) と数えます。

## 使用例

[コンパイル可能な使用例](../../examples/static_query/static_range_lis.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../static_query/static_range_lis.hpp"
int main(){
    cp::StaticRangeLIS<int> lis({3,1,2,2,4,0,5});
    std::cout<<lis.query(0,7)<<' '<<lis.query(1,5)<<' '<<lis.query(3,3)<<'\n';
}
```

```text
4 3 0
```

## 使用する問題

[Library Checker: Static Range LIS Query](https://judge.yosupo.jp/problem/static_range_lis_query)。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[tko919/libraryのRangeLIS](https://github.com/tko919/library/blob/3925f75892c0705fb22d6e3f892ae1b5b4fac32d/DataStructure/rangelis.hpp)（CC0）を改変。固定commit、ライセンスとローカルhashは[出典台帳](../../verification/provenance.json)に記録。マクロ・外部依存を除き、重複値・汎用Compare・空入力・const APIを追加しました。[ko_osagaの解説](https://codeforces.com/blog/entry/111807)も参照。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
