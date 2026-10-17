---
title: 2-SAT
documentation_of: //graph_connectivity/two_sat.hpp
---

# 2-SAT

二つのリテラルからなる節を満たす真偽値割当を取得。

## API・使用条件

TwoSAT(n)、add_clause(i,f,j,g)、force(i,value)、imply(i,f,j,g)、solve() const -> optional<vector<bool>>。

add_clauseは(x_i=f) OR (x_j=g)、implyは(x_i=f)なら(x_j=g)。0<=n<=100000000。不成立nullopt、n=0は成功した空列。追記/再solve可、割当は非一意。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_connectivity/two_sat.hpp"
int main() {
    cp::TwoSAT s(2);s.force(0,true);s.imply(0,true,1,true);
    auto answer=s.solve(); for(bool x:*answer) { std::cout<<x; } std::cout<<"\n";
}
```

出力: `11`。

## 原理・計算量

各節を二つの含意へ写し、既存の反復Kosarajuを共有する。ある変数の真/偽が同じSCCなら不成立。SCCのトポロジカル順から割り当てる。

構築O(N)、節追加O(1)、solve O(N+M)、領域O(N+M)。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#two_sat) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_connectivity/two_sat.cpp) / [カテゴリ索引](README.md)
