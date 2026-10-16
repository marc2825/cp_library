---
title: Persistent Segment Tree
documentation_of: //ordered_structure/persistent_segment_tree.hpp
---

# Persistent Segment Tree

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#persistent) / [実装](../../ordered_structure/persistent_segment_tree.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "ordered_structure/persistent_segment_tree.hpp"
long long op(long long a, long long b) { return a + b; }
long long e() { return 0; }
int main() {
    cp::PersistentSegmentTree<long long, op, e> s(4);
    int a = s.set(0, 1, 5), b = s.set(a, 2, 7), c = s.set(a, 1, 9);
    std::cout << s.prod(a, 0, 4) << ' ' << s.prod(b, 0, 4) << ' ' << s.prod(c, 0, 4) << '\n';
}
```

[実行できる使用例](../../examples/ordered_structure/persistent_segment_tree.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`PersistentSegmentTree<S,op,e>(n)`。`empty_root()==0` は全要素e()。`set(root,p,x)->new_root`、`prod(root,l,r)`、`get(root,p)`、`size/node_count`。op/eは既存Segtreeと同じモノイド、非可換可。版を更新するたびrootを利用側で保存します。過去の任意版から分岐可能。

rootはこのオブジェクトが返したrootだけを使用し、内部ノード番号を推測して渡してはいけません。全体コピー後はコピー時に存在したrootも同じ番号で使えます。不要版の個別解放はしません。n≥0、添字はassert、格納ノード数はintの範囲。

## 計算量・原理

初期構築O(1)、set/prod/getはO(log(n+1))、U回更新でO(1+U log(n+1))領域。Sのコピー・opの費用を掛ける。

更新位置までの経路だけを複製し、それ以外を旧版と共有します。0番ノードは全てe()の未展開区間を表し、巨大なnでも初期配列の確保を不要にします。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/persistent_unionfind)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
