---
title: Min-Cost b-Flow
documentation_of: //graph_flow/min_cost_b_flow.hpp
---

# Min-Cost b-Flow

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#min_cost_b_flow) / [実装](../../graph_flow/min_cost_b_flow.hpp) / [一次資料と設計](../b_7_11/references.md)

各頂点の流出−流入を指定し、辺の下限・上限を満たす最小費用流を求めます。負費用辺を上限まで流してから、残余網で不均衡を解消します。

## usage

```cpp
#include <iostream>
#include "graph_flow/min_cost_b_flow.hpp"
int main() {
    cp::MinCostBFlow<> f(2); f.set_balance(0,2); f.set_balance(1,-2); f.add_edge(0,1,0,3,-4);
    auto r=f.solve(); std::cout << cp::optimization_to_string(r->cost) << ' ' << r->flow[0] << '\n';
}
```

出力:

```text
-8 2
```

## API・使用条件

`MinCostBFlow<Cap=long long,Cost=long long>(n)`。`set_balance(v,b)`、`add_edge(u,v,lower,upper,cost)`、`solve()`→optional<Result>。Cap/Costは符号付き64bit以下の整数。Resultは符号付き128bitのcost、各辺flow、128bit potential。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

b>0は供給、b<0は需要。負のlower/upper、ループ、負費用閉路も有限容量内で最適化します。初期残余費用は非負、以後はpotentialを使用。最後に全成分の双対証明をBellman–Fordで作ります。残余辺のcost+p[u]−p[v]>=0を保証。実行不能はnullopt、128bit途中超過はoverflow_error。solveは入力を消費せず再実行可。

## 計算量

A回の増加に対しO(A(V+E) log(V+E+1)+VE)、領域O(V+E)。容量値に依存するSSP法で、強多項式時間は保証しない。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 各頂点の流出−流入を指定し、辺の下限・上限を満たす最小費用流を求めます。負費用辺を上限まで流してから、残余網で不均衡を解消します。 [実行できる小問題](../../examples/graph_flow/min_cost_b_flow_b.cpp)。

関連する出題: [Minimum Cost b-flow](https://judge.yosupo.jp/problem/min_cost_b_flow)。入出力仕様の参照先であり、オンラインACの主張ではありません。

Min-Cost b-Flowを必要とする3.9の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
