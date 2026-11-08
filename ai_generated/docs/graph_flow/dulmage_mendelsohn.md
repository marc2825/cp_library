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

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

block0は右余剰、最後は左余剰で空でも保持します。中央の均衡blockはtopological順。元辺L→Rのcomponentは非減少。allowed_edgeは端点が同じblockに属すかで判断し、最大マッチングの全列挙と比較しています。

## 計算量

O(E sqrt(V)+V+E)時間、O(V+E)領域。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 最大二部マッチングから交互有向グラフを作り、左右の余剰領域と均衡したSCCを分解します。各辺が何らかの最大マッチングで使用可能かも返します。 [実行できる小問題](../../examples/graph_flow/dulmage_mendelsohn_b.cpp)。

DM分解を必要とする3.9の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
