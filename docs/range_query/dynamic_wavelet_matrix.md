---
title: Dynamic Wavelet Matrix
documentation_of: //range_query/dynamic_wavelet_matrix.hpp
---

# Dynamic Wavelet Matrix（D-28）

uint64列への点代入・途中挿入・削除を行いながら、頻度・k番目・rank/selectを答えます。各bit段の列をDynamicBitVectorで保持し、安定分割先の位置を更新します。登録していない値への変更も可能です。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#dynamic_wavelet_matrix)

## API・使用条件

`DynamicWaveletMatrix(bit_width=64)` / `(vector<uint64_t>,bit_width=64)`。

- `insert(p,x)`, `erase(p)` → 消した値、`set(p,x)`, `access(p)`。
- `count(l,r,x)`, `rank(x,end)`, `count_less(l,r,upper)`, `range_freq(l,r,lower,upper)`。
- `kth_smallest(l,r,k)` → optional。`select(x,k)` → 全列で0番目から数えるk番目の位置、なければ-1。
- `predecessor(l,r,upper)` はupper未満の最大、`successor(l,r,lower)` はlower以上の最小。
- `size()`, `empty()`, `bit_width()`。

bit幅1..64以外はinvalid_argument。値が幅を超える更新はout_of_range。B=64でも値0とUINT64_MAXを扱えます。半開値域の上端に2^64は渡せないため全域の個数は区間長を使います。メモリ不足時の複数段更新について強い例外保証はありません。

## 計算量・保存領域

bit幅Bで構築期待 O(nB log n)、各操作期待 O(B log n)、領域 O(BN) words。Nは各bit列が保持する最大同時要素数で、削除ノードを再利用します。圧縮bitvectorではなく各bitに木ノードを使うため、特にB=64ではメモリ消費が大きくなります。

## 使用例

[コンパイル可能な使用例](../../examples/range_query/dynamic_wavelet_matrix.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../range_query/dynamic_wavelet_matrix.hpp"
int main(){
    cp::DynamicWaveletMatrix matrix(std::vector<std::uint64_t>{5,1,4,1},3);
    matrix.insert(2,7); matrix.set(0,2);
    std::cout<<matrix.rank(1,5)<<' '<<*matrix.kth_smallest(0,5,2)<<' '<<matrix.select(1,1)<<'\n';
    auto removed=matrix.erase(2);
    std::cout<<removed<<' '<<matrix.range_freq(0,4,1,4)<<'\n';
}
```

```text
2 2 4
7 3
```

## 使用する問題

Dynamic Sequence Range Kth / Frequency。値の座標圧縮を事前に決められない列更新。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Claude–Navarro–Ordóñez, The Wavelet Matrix](https://doi.org/10.1016/j.is.2014.06.002)。各段のrank/selectを今回の動的ビット列に置き換えました。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
