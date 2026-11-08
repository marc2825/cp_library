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

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

各頂点の自己tokenにだけ値を置き、各辺に値が単位元の2tokenを置きます。切断後のtokenは再利用せずオブジェクト破棄まで保持します。巡回列の回転で積の順序が変わるため可換性が必要。一般グラフの辺削除や部分木積は対象外。

## 計算量

各操作期待O(log(V+1))、領域O(V+成功したlink総数)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 森の各木を巡回Euler列としてimplicit treapに載せ、列の回転・連結・切断からlink/cutを処理します。 [実行できる小問題](../../examples/tree/euler_tour_tree_b.cpp)。

Euler Tour Treeを必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
