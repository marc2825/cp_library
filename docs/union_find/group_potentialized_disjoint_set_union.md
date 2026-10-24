---
title: 非可換群上のPotentialized DSU
documentation_of: //union_find/group_potentialized_disjoint_set_union.hpp
---

# 非可換群上のPotentialized DSU（D-20）

頂点の群値を `P[v]` とし、`inverse(P[u]) * P[v] = w` という制約を追加します。親から子への差を保存し、経路圧縮時にも積の順序を保持します。加法専用の既存DSUと結果enumを共有します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#group_potentialized_disjoint_set_union)

## API・使用条件

`GroupPotentializedDisjointSetUnion<G>(n)`。`G::S`, `G::e()`, `G::op(a,b)`, `G::inverse(a)` を指定します。

- `merge(u,v,w)` は `PotentialMergeResult::{Merged,AlreadyConsistent,Contradiction}`。
- `diff(u,v)` は連結なら `optional<S>`、非連結なら空。
- `potential(v)` は代表元からの差。代表元変更に伴って変わります。
- `leader(v)`, `same(u,v)`, `size(v)`, `vertex_count()`, `component_count()`。

群の結合則・単位元・逆元と `S::operator==` が必要です。添字の条件はassert。頂点の絶対ポテンシャルは固定しません。矛盾した制約は採用しません。

## 計算量・保存領域

構築 O(n)、各操作ならし O(α(n))、領域 O(n)。群演算・等値比較を O(1) と数えます。

## 使用例

[コンパイル可能な使用例](../../examples/union_find/group_potentialized_disjoint_set_union.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../union_find/group_potentialized_disjoint_set_union.hpp"
#include <array>
struct Group {
    using S=std::array<int,3>;
    static S e(){return {0,1,2};}
    static S op(S a,S b){return {a[b[0]],a[b[1]],a[b[2]]};}
    static S inverse(S a){S b{};for(int i=0;i<3;++i)b[a[i]]=i;return b;}
};
int main(){
    cp::GroupPotentializedDisjointSetUnion<Group> dsu(3);
    dsu.merge(0,1,{1,0,2}); dsu.merge(1,2,{0,2,1});
    auto p=*dsu.diff(0,2);
    std::cout<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';
}
```

```text
1 2 0
```

## 使用する問題

置換・行列・座標変換の整合性制約、非可換群上の差分問い合わせ。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Tarjan, union-findの解析](https://doi.org/10.1145/321879.321884)。非可換の差の向きは上記の式で定義した独自実装です。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
