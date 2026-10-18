---
title: Mongeの辺数制約付き最短路
documentation_of: //dp_optimization/monge_shortest_path_k_edges.hpp
---

# Mongeの辺数制約付き最短路

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#monge_shortest_path_k_edges) / [実装](../../dp_optimization/monge_shortest_path_k_edges.hpp) / [一次資料と設計](../b_12_15/references.md)

ちょうどK辺のDP各段をSMAWKで計算し、K本の辺からなるパスを復元します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/monge_shortest_path_k_edges.hpp"
int main() {
    auto r=cp::monge_shortest_path_k_edges(5,2,[](int i,int j){return 1LL*(j-i)*(j-i);});
    std::cout << r->cost << ' ' << r->vertices.size()-1 << '\n';
}
```

出力:

```text
13 2
```

## API・使用条件

`monge_shortest_path_k_edges(n,k,cost)`→optional<MongeKEdgePathResult>{cost,vertices}。頂点0..n、全i<jに有限long longのMonge費用。n,k>=0、K>NまたはK=0,N>0はnullopt。N=K=0は{0,{0}}。

## 仕組み・境界条件

各段の全単調な行最小値を一括計算する方式です。罰則の二分探索ではないため、探索範囲や同点の辺数の選び方を指定する必要はありません。Nが大きくKも大きい場合は時間・復元表を見積もってください。Monge条件は利用者の前提、加算超過はoverflow_error。

## 計算量

O(KN)回のcost評価と時間、復元表の領域O(KN)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

区間をちょうどK個へ分割するDP。 [動く小問題](../../examples/dp_optimization/monge_shortest_path_k_edges_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
