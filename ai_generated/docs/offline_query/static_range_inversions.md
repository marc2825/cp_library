---
title: 静的転倒数クエリ
documentation_of: //offline_query/static_range_inversions.hpp
---

# 静的転倒数クエリ（D-27）

既存のMo・FenwickTree・CoordinateCompressionを組み合わせた転倒数計算を、使用側から呼べる公開関数へ抽出しました。区間ごとの `i<j` かつ `compare(a[j],a[i])` を満たす組数を返します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#static_range_inversions)

## API・使用条件

`static_range_inversions(values,queries,compare={},block_size=0)` → `vector<long long>`。queriesは `vector<pair<int,int>>` の半開区間。返却順は入力順です。

Compareはstrict weak ordering。同値は転倒に含めません。全問い合わせを先に渡すオフラインAPIです。添字はMoのassert条件、組数はlong longに収まること。

## 計算量・保存領域

ブロック幅B、問い合わせ数qで O(n log n+q log q+(n²/B+qB) log n)、追加領域 O(n+q)。既定幅は既存Moの選択に従います。

## 使用例

[コンパイル可能な使用例](../../examples/offline_query/static_range_inversions.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../offline_query/static_range_inversions.hpp"
int main(){
    auto result=cp::static_range_inversions(std::vector<int>{3,1,2,1},{{0,4},{1,4},{2,2}});
    for(auto count:result)std::cout<<count<<' ';
    std::cout<<'\n';
}
```

```text
4 1 0
```

## 使用する問題

[Library Checker: Static Range Inversions Query](https://judge.yosupo.jp/problem/static_range_inversions_query)。既存のジャッジ形式ラッパーもこの公開関数を呼びます。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

既存 [Mo](../offline_query/mo.md) と [Fenwick Tree](../range_query/fenwick_tree.md) を共有。右端と左端では増減する転倒の方向が逆であることを4つのcallbackへ明記しています。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
