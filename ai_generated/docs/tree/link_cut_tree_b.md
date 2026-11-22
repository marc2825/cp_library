---
title: Link-Cut Tree（B項目）
documentation_of: //tree/link_cut_tree.hpp
---

# Link-Cut Tree（B項目）

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#link_cut_tree) / [実装](../../tree/link_cut_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

既存Link-Cut Treeを共有し、動的な森上のlink/cut・頂点更新・方向付きパス積を扱います。

## usage

```cpp
#include <iostream>
#include "tree/link_cut_tree.hpp"
long long sum(long long a,long long b){return a+b;}
long long identity(){return 0;}
int main() {
    cp::LinkCutTree<long long,sum,identity> t(std::vector<long long>{1,2,3});
    t.link(0,1); t.link(1,2); std::cout << *t.path_prod(0,2) << ' ' ;
    t.cut(1,2); std::cout << t.connected(0,2) << '\n';
}
```

出力:

```text
6 0
```

## API・使用条件

`LinkCutTree<S,op,e>`、`link,cut,connected,make_root,set,get,path_prod,kth_on_path`。opはモノイドで可換でなくてもよい。path_prodは非連結ならnullopt。

頂点 ID は `[0,size())`（assert）。Graph は受け取らず、`add_vertex` で末尾に頂点を追加できます。

## 仕組みと境界

既存[詳細説明](link_cut_tree.md)も参照。辺の値は辺用の頂点を追加して管理します。linkは閉路を作るとfalse、cutは実在の直接辺でなければfalse。Euler Tour Treeの成分集約とは用途が異なります。

## 計算量

各操作ならしO(log(V+1))、領域O(V)。 Vは頂点数。op・e と S のコピーをO(1)として数えています。

## 使用する問題・検証

想定問題: 辺の追加・削除を含む森で、頂点値の更新と u→v パスの積（非可換でもよい）を答える問題（例: [Dynamic Tree Vertex Set Path Composite](https://judge.yosupo.jp/problem/dynamic_tree_vertex_set_path_composite)）。 [実行できる小問題](../../examples/tree/link_cut_tree_b.cpp)。

検証では、1〜30頂点の森100個×600操作で、link/cut の可否・connected・make_root 後の文字列連結による u→v のパス積・kth_on_path を BFS と照合します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/tree_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
