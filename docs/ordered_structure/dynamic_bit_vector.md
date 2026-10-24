---
title: Dynamic Rank/Select Bit Vector
documentation_of: //ordered_structure/dynamic_bit_vector.hpp
---

# Dynamic Rank/Select Bit Vector（D-29）

要仕様化だったD-29を「挿入・削除可能なrank/selectビット列」と定義しました。implicit treapに部分木長と1の数を持たせ、0/1それぞれのprefix頻度とk番目の位置を探索します。動的Wavelet Matrixから共有します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#dynamic_bit_vector)

## API・使用条件

`DynamicBitVector()` / `(vector<bool>)`。`insert(p,bit)`, `erase(p)` → 消したbit、`set(p,bit)`, `access(p)`、`rank(bit,end)` → `[0,end)` の個数、`select(bit,k)` → k番目の位置（0-based）、見つからなければ-1。`size()`, `empty()`, `clear()`。

添字はassert。負のkや存在しない出現のselectは-1。コピーは深いコピーで O(N)、元と独立に更新できます。乱数は固定seedで再現可能です。

## 計算量・保存領域

構築 O(n log n)、全操作期待 O(log n)。領域 O(N) words、Nは過去の最大同時要素数。削除ノードはfree listで再利用します。succinctな圧縮ビット列ではありません。clearは保持ノードを破棄しますがvector容量は残る場合があります。

## 使用例

[コンパイル可能な使用例](../../examples/ordered_structure/dynamic_bit_vector.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../ordered_structure/dynamic_bit_vector.hpp"
int main(){
    cp::DynamicBitVector bits(std::vector<bool>{1,0,1,1});
    bits.insert(1,true); bits.erase(3);
    std::cout<<bits.rank(true,4)<<' '<<bits.select(false,0)<<' '<<bits.select(true,2)<<'\n';
}
```

```text
3 2 3
```

## 使用する問題

動的ビット列、挿入可能な文字列のrank/select、Dynamic Wavelet Matrixの内部。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Seidel–Aragon, Randomized Search Trees](https://doi.org/10.1007/s004539900061)。部分木の長さ・1の個数を付加した独自実装です。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
