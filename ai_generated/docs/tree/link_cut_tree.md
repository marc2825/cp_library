---
title: Link-Cut Tree
documentation_of: //tree/link_cut_tree.hpp
---

# Link-Cut Tree

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#lct) / [実装](../../tree/link_cut_tree.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include <string>
#include "tree/link_cut_tree.hpp"
std::string op(std::string a, std::string b) { return a + b; }
std::string e() { return {}; }
int main() {
    cp::LinkCutTree<std::string, op, e> t(std::vector<std::string>{"a", "b", "c"});
    t.link(0, 1); t.link(1, 2); std::cout << *t.path_prod(2, 0) << ' ';
    t.cut(1, 2); std::cout << t.connected(0, 2) << '\n';
}
```

[実行できる使用例](../../examples/tree/link_cut_tree.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`LinkCutTree<S,op,e>(n)`（初期e()）または `(vector<S>)`。`link(u,v)->bool` は異なる木を接続、閉路になるならfalse。`cut(u,v)->bool` は直接の辺だけ削除。`connected`、`set/get(v)`、`path_prod(u,v)->optional<S>`、`kth_on_path(u,v,k)->int`、`make_root(v)`、`size()`。パスはu→vの順で非可換モノイド可。別成分のprodはnullopt、k範囲外や別成分のkthは−1。

自己ループのlink/cutはfalse。辺の値は辺用の頂点を用意し、その両端へlinkして表現します。部分木集約・パス一括更新はこのAPIには含めません。make_root以外の操作も内部の代表根を変えるので、固定根付き木の親を別途仮定しないでください。

## 計算量・原理

構築O(n)、各操作ならしO(log n)、単一操作は最悪O(n)、格納O(n)。Sのコピーとopの費用を掛ける。splay祖先のpushも反復処理で深い鎖に対応。

優先パスをsplay木で表し、accessで目的頂点までのパスを露出します。make_rootはパス反転。各補助木は順方向と逆方向の積を持ち、パスの向きが変わっても非可換演算を正しく扱います。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/dynamic_tree_vertex_add_path_sum)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。

2026-10-25: `add_vertex(value=e()) -> int`で孤立頂点を末尾に追加できます。既存IDは安定、vector再確保を含めならしO(1)。[増分MSF](../graph_basic/incremental_msf.md)では辺ノードの作成・再利用に使います。
