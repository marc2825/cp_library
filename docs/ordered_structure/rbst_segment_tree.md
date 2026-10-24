---
title: RBST-based Segment Tree
documentation_of: //ordered_structure/rbst_segment_tree.hpp
---

# RBST-based Segment Tree（D-23）

要仕様化だったD-23を「挿入・削除できる列のモノイド区間積」と定義しました。左右のサイズ比で根を選ぶ実際のRBSTを用い、PersistentLazyRBSTに恒等作用を与えて実装を共有します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#rbst_segment_tree)

## API・使用条件

`RBSTSegmentTree<S,op,e>(values={},seed)`。

- `insert(p,x)`, `erase(p)`, `set(p,x)`, `get(p)`。
- `prod(l,r)`, `all_prod()`, `reverse(l,r)`。
- `move(l,r,p)` は区間を取り出した後の列の位置pへ挿入。
- `split(p)` → 元を保持して2列。`concat(other)` → 元を保持して結合した列。
- `to_vector()`, `size()`, `empty()`, `clear()`。

0-based・半開区間、添字はassert。非可換monoidも可。コピーは独立に更新でき、乱数seedもコピーします。モノイド演算は純粋な関数、合計長はintに収まること。構造共有を残さなければ不要ノードは自動解放します。

## 計算量・保存領域

構築 O(n log n)、更新・区間積・split/concat・moveは期待 O(log n)、all_prod・size・コピーは O(1)。要素演算を O(1) と数えます。生存する1版は O(n)、分岐版が保持する更新経路は1操作当たり期待 O(log n) ノード。最後の参照を消すclear/破棄は解放数に比例します。

## 使用例

[コンパイル可能な使用例](../../examples/ordered_structure/rbst_segment_tree.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../ordered_structure/rbst_segment_tree.hpp"
#include <string>
std::string op(std::string a,std::string b){return a+b;}
std::string e(){return {};}
int main(){
    cp::RBSTSegmentTree<std::string,op,e> tree({"a","b","c","d"});
    tree.insert(2,"X"); tree.reverse(1,4);
    std::cout<<tree.all_prod()<<' '<<tree.prod(1,4)<<'\n';
    auto [left,right]=tree.split(3);
    std::cout<<right.concat(left).all_prod()<<'\n';
}
```

```text
acXbd cXb
bdacX
```

## 使用する問題

挿入・削除・反転を含むDynamic Sequence Range Aggregate。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Martínez–Roura, Randomized Binary Search Trees](https://doi.org/10.1145/274787.274812)。固定priorityではなくサイズ比によるjoinを実装します。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
