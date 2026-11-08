---
title: Measured Persistent Finger Tree
documentation_of: //ordered_structure/finger_tree.hpp
---

# Measured Persistent Finger Tree（D-30）

1〜4要素の左右digitと、2/3要素Nodeを持つ再帰的middleからなる実際のFinger Treeです。要素数と順序付きmonoid積を計測し、両端操作・連結・位置での分割を行います。全ノードが不変で、コピーから過去版を分岐できます。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#finger_tree)

## API・使用条件

`FingerTree<S,op,e>(values={})`。

- `push_front/back(x)`, `pop_front/back()` → 消した値、`front/back()`。
- `get(p)`, `prod(l,r)`, `all_prod()`。
- `split(p)` → 元を残すpair、`concat(other)` → 元を残す新しい列。
- `to_vector()`, `size()`, `empty()`, `clear()`。

非可換monoidに対応。空のfront/back/popと範囲外添字はassert。共有参照で過去版を保存します。論文の遅延評価をそのまま使わないため、完全永続環境での両端ならし O(1) は主張しません。

## 計算量・保存領域

コピー・size・front/back・all_prodは O(1)。split/concat/get/prodは O(log(n+1)) の上界、両端push/popもstrict評価で最悪 O(log(n+1)) とします。構築の安全な上界は O(n log(n+1))。1版 O(n)、保存した変更経路は1操作 O(log n)。最後の参照を解放する時間は解放ノード数に比例します。monoid演算を O(1) と数えます。

## 使用例

[コンパイル可能な使用例](../../examples/ordered_structure/finger_tree.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../ordered_structure/finger_tree.hpp"
#include <string>
std::string op(std::string a,std::string b){return a+b;}
std::string e(){return {};}
int main(){
    using Tree=cp::FingerTree<std::string,op,e>;
    Tree original({"a","b","c"}),tail({"x","y"});
    auto [left,right]=original.concat(tail).split(2);
    std::cout<<left.all_prod()<<' '<<right.all_prod()<<'\n';
    right.pop_front();
    std::cout<<right.all_prod()<<' '<<original.all_prod()<<'\n';
}
```

```text
ab cxy
xy abc
```

## 使用する問題

永続Deque、順序付き集約を持つ列の連結・分割。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Hinze–Paterson, Finger trees: a simple general-purpose data structure](https://www.staff.city.ac.uk/~ross/papers/FingerTree.html)。著者の訂正にある遅延評価の重要性も踏まえ、C++版の保証を区別します。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
