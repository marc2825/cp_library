---
title: Cost Scaling Min-Cost b-Flow
documentation_of: //graph_flow/cost_scaling_b_flow.hpp
---

# Cost Scaling Min-Cost b-Flow

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#cost_scaling_b_flow) / [実装](../../graph_flow/cost_scaling_b_flow.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

上下限・負費用・頂点の供給需要を含む最小費用流を、整数epsilon cost scalingで解きます。既存SSPと別の方式です。

## usage

```cpp
#include <iostream>
#include "graph_flow/cost_scaling_b_flow.hpp"
int main() {
    cp::CostScalingBFlow<> f(2);
    f.set_balance(0,2); f.set_balance(1,-2);
    f.add_edge(0,1,0,3,-4);
    auto r=f.solve();
    std::cout << cp::optimization_to_string(r->cost) << ' ' << r->flow[0] << '\n';
}
```

出力:

```text
-8 2
```

## API・使用条件

`CostScalingBFlow<Cap=long long,Cost=long long>(n)`、`set_balance(v,net_out)`、`add_edge(u,v,lower,upper,cost)`→元辺ID、`edge_count()`、`solve()`→`optional<Result>`。Resultは128bit `cost`、入力順の `flow`、128bit `potential`。Cap/Costは符号付き64bit以下。

0始まり。正のbalanceは流出超過。負のlower/upper、自己ループ、多重辺、非連結成分を許します。実行不能はnullopt。頂点・下限上限の不正は例外、検査付き128bit演算の超過はoverflow_error。solveは再実行可能。残余辺すべてで `cost+p[u]-p[v]>=0` が成り立つ双対証明を返します。

## 仕組み

Dinicで上下限付きb-flowの実行可能解を作り、費用をn+1倍します。負の被約費用の残余辺を飽和してから、excessのpushとpotentialのrelabelで実行可能性を回復します。epsilonを半減し1まで進めると整数費用の負閉路がなくなります。最後に元の費用で全成分Bellman–Fordを実行して双対を復元します。

## 計算量

O(n²m log(nC+2))時間、O(n+m)領域。Cは最大絶対費用。実行可能性のDinic O(n²m)と双対復元O(nm)を含む。容量値に比例する回数のSSPは行いません。実行可能性の Dinic は再帰しないので、長い鎖でもスタックは溢れません。

## 使用する問題・検証

容量が大きく、負費用循環も含む輸送・割当。 [Minimum Cost b-flow](https://judge.yosupo.jp/problem/min_cost_b_flow) の形式に接続できます。

n≤5・m≤7 の1,000例（7例に1例は balance の総和≠0）を全ての流量割当の列挙と比較し、最小費用・上下限・保存則・被約費用の符号・解き直しの一致を確かめます。n=30・m=250・容量約10^12・balance 0 の40例では MinCostBFlow と費用を比較し、両方の証明（上下限・保存則・被約費用）を確かめます。20万頂点の鎖も確認します。 [実行できる使用例](../../examples/graph_flow/cost_scaling_b_flow.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
