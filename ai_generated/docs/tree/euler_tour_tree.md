---
title: Euler Tour Tree
documentation_of: //tree/euler_tour_tree.hpp
---

# Euler Tour Tree

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#euler_tour_tree) / [実装](../../tree/euler_tour_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

森の各木を巡回Euler列としてimplicit treapに載せ、列の回転・連結・切断からlink/cutを処理します。

## usage

```cpp
#include <iostream>
#include "tree/euler_tour_tree.hpp"
long long sum(long long a,long long b){return a+b;}
long long identity(){return 0;}
int main() {
    cp::EulerTourTree<long long,sum,identity> t(std::vector<long long>{1,2,3});
    t.link(0,1); t.link(1,2); std::cout << t.component_prod(2) << ' ';
    t.cut(0,1); std::cout << t.component_prod(2) << '\n';
}
```

出力:

```text
6 5
```

## API・使用条件

`EulerTourTree<S,op,e>(valuesまたはn)`、`link,cut,connected,make_root,set,get,component_prod,component_size,allocated_nodes`。opは可換モノイド。

頂点 ID は `[0,n)`（assert）で、頂点数は構築時に固定です。負の n は`invalid_argument`。`op`・`e` は `S(*)(S,S)`・`S(*)()` 型の関数ポインタで渡します。

## 仕組みと境界

各頂点の自己tokenにだけ値を置き、各辺に値が単位元の2tokenを置きます。切断後のtokenは再利用せずオブジェクト破棄まで保持します。巡回列の回転で積の順序が変わるため可換性が必要。一般グラフの辺削除や部分木積は対象外。

## 計算量

各操作期待O(log(V+1))、領域O(V+成功したlink総数)。 Vは頂点数。op・e と S のコピーをO(1)として数えています。

## 使用する問題・検証

想定問題: 辺の追加・削除を含む森で、連結判定と成分の値の和・頂点数を答える問題（[Dynamic Tree Vertex Add Subtree Sum](https://judge.yosupo.jp/problem/dynamic_tree_vertex_add_subtree_sum) は `cut(v,p)`→`component_prod(v)`→`link(v,p)` で解けます。辺 token は再利用しないので、操作数に比例してメモリが増えます）。 [実行できる小問題](../../examples/tree/euler_tour_tree_b.cpp)。

検証では、同じ100個×600操作で、link/cut の可否・connected・component_prod・component_size を BFS の成分と照合します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/tree_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
