---
title: Push-Relabel
documentation_of: //graph_flow/push_relabel.hpp
---

# Push-Relabel

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#push_relabel) / [実装](../../graph_flow/push_relabel.hpp) / [一次資料と設計](../b_7_11/references.md)

FIFOのactive頂点を処理し、超過流と高さを使って最大流を求めます。各論理辺の流量と最小cutも返します。

## usage

```cpp
#include <iostream>
#include "graph_flow/push_relabel.hpp"
int main() {
    cp::PushRelabel<> f(3); f.add_edge(0,1,5); f.add_edge(1,2,3); f.add_edge(0,2,1);
    std::cout << f.solve(0,2).value << '\n';
}
```

出力:

```text
4
```

## API・使用条件

`PushRelabel<Cap=long long>(n)`、`add_edge(u,v,cap)`、`solve(s,t)`→{value,flow,source_side}、`max_flow(s,t)`。非負整数容量64bit以下、s!=t。

頂点番号・s≠t は assert による使用条件、負の容量は`invalid_argument`です。

## 仕組みと境界

各solveは流量0からの再計算です。Dinicの継続増加APIとは異なります。sourceの出容量合計がCapを超えても超過流を128bitで保持し、最終流量がCapを超えた場合だけoverflow_error。自己ループ・平行辺可。

## 計算量

一般的なFIFO版の保守的上界O(V²E)、領域O(V+E)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: 密なグラフや平行辺の多いネットワークの最大流と、最小カット（`source_side`）の復元。 [実行できる小問題](../../examples/graph_flow/push_relabel_b.cpp)。

検証では、頂点2〜9・辺34本以下・容量0〜8 の800例で値を Dinic と比較し、流量の範囲・保存則・`source_side` のカット容量が値に等しいことを確かめます。`unsigned long long` の最大容量の平行辺も確認します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/flow_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
