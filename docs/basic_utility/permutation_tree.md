---
title: 順列木・共通区間分解
documentation_of: //basic_utility/permutation_tree.hpp
---

# 順列木・共通区間分解（U-08）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#permutation_tree)

右端を進めながら単調スタックで区間の最大・最小の変化を追い、区間加算・最小値セグメント木で共通区間になれる左端を検出します。隣接値域の結合とPrime節点の形成を繰り返します。hitonanodeのMIT実装を既存LazySegtreeとcp規約に適合させました。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/permutation_tree.hpp"
int main() {
    cp::PermutationTree tree({1, 0, 3, 5, 2, 4});
    const auto& root = tree.nodes()[tree.root()];
    std::cout << root.left << ' ' << root.right << ' ' << root.children.size() << '\n';
    std::cout << tree.nodes().size() << ' ' << tree.common_interval_count() << '\n';
}
```

出力:

```text
0 6 2
9 9
```

## API・使用条件

`PermutationTree(vector<int>)` の入力は `[0,n)` の置換。不正入力は `invalid_argument`、n>10^8は `length_error`（依存先セグメント木の上限、実用可能なメモリ量を保証する値ではありません）。空なら `root()==-1`、nodesは空。

`nodes()` はNodeのconst配列、`root()` は根ID。Nodeは `type,left,right,low,high,parent,children` を持ち、位置は `[left,right)`、値域も `[low,high)`。parentは根のみ−1。childrenは位置順で、**ID順・生成順が後順走査であるとは限りません**。

- `Leaf`：要素1個。
- `Increasing` / `Decreasing`：子の値域がそれぞれ昇順／降順で隣接。連続する子を任意個まとめた区間が共通区間です。
- `Prime`：子を2個以上、全体未満でまとめても共通区間になりません。

節点は他の共通区間と交差（片方が他方を含む場合を除く）しない「強共通区間」です。全共通区間は節点自身とIncreasing/Decreasing節点の連続する子の和として表されます。`common_interval_count()` は非空の共通区間の個数。Cartesian Treeではありません。

## 計算量

構築 O(n log n)、記憶O(n)、節点数は非空入力で高々2n−1。個数計算はO(n)。全共通区間を明示列挙すると単調順列でΘ(n²)件になります。構築に木の深さに比例する再帰は使用しません。

## 使用する問題・検証

Library Checker Common Interval Decomposition Tree、順列の連続値区間数、共通区間木上のDP。n≤8の全順列で全区間から強区間を独立に抽出し、節点集合・型・親子・表現できる全区間・個数を比較。20万要素の昇順・降順・シャッフル列でも構築します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
