---
title: DM分解
documentation_of: //graph_flow/dulmage_mendelsohn.hpp
---

# DM分解

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dulmage_mendelsohn) / [実装](../../graph_flow/dulmage_mendelsohn.hpp) / [一次資料と設計](../b_7_11/references.md)

最大二部マッチングから交互有向グラフを作り、左右の余剰領域と均衡したSCCを分解します。各辺が何らかの最大マッチングで使用可能かも返します。

## usage

```cpp
#include <iostream>
#include "graph_flow/dulmage_mendelsohn.hpp"
int main() {
    auto d=cp::dulmage_mendelsohn(2,2,{{0,0},{0,1},{1,1}});
    for(bool x:d.allowed_edge) { std::cout << x; }
    std::cout << '\n';
}
```

出力:

```text
101
```

## API・使用条件

`dulmage_mendelsohn(L,R,vector<pair<int,int>> edges)`→DulmageMendelsohnResult。`matching,component,blocks,allowed_edge`。頂点は左[0,L)、右[L,L+R)。多重辺可。

端点の範囲は assert による使用条件です。

## 仕組みと境界

block0は右余剰、最後は左余剰で空でも保持します。中央の均衡blockはtopological順。元辺L→Rのcomponentは非減少。allowed_edgeは端点が同じblockに属すかで判断し、最大マッチングの全列挙と比較しています。

## 計算量

O(E sqrt(V)+V+E)時間、O(V+E)領域。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: 各辺がある最大マッチングに入り得るかの判定、完全マッチングを持つ部分への分解、疎行列の構造的ランクとブロック三角化。 [実行できる小問題](../../examples/graph_flow/dulmage_mendelsohn_b.cpp)。

検証では、辺彩色と同じ1,200例で全辺部分集合を列挙し、最大マッチングの大きさ、`allowed_edge` が全ての最大マッチングの和集合に一致すること、各辺で `component[u]≤component[L+v]` を確かめます（`blocks` の中身は直接は検査していません）。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/flow_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
