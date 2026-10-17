---
title: Functional Graph
documentation_of: //graph_basic/functional_graph.hpp
---

# Functional Graph

各頂点の遷移先が一つのグラフを、閉路と流入木に分解。

## API・使用条件

FunctionalGraph(vector<int> next)、jump(v,uint64_t steps)、min_steps(u,v)->optional<uint64_t>、on_cycle(v)、size/component_count。公開配列next/component/depth/entry/position/cycles。

nextは全て[0,N)、-1終端を許容しない。空も可。depthは閉路までの辺数、entryは最初に入る閉路頂点、positionはentryの閉路内位置（閉路上では自身の位置）。cyclesは遷移順。到達不能のmin_stepsはnullopt。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/functional_graph.hpp"
int main() {
    cp::FunctionalGraph f({1,2,1,2});
    std::cout<<f.jump(3,4)<<" "<<*f.min_steps(0,2)<<"\n";
}
```

出力: `1 2`。

## 原理・計算量

入次数0を剥がして残った閉路を列挙し、剥がした逆順に流入木へ情報を配る。木の頂点への距離は深さ差のjumpで確認し、閉路上の頂点へは深さ+閉路上の位置差を返す。jumpは既存BinaryLiftingを共有。

分解O(N)、BinaryLiftingは64段でO(64N)時間・領域。jump/min_stepsはO(64)。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#functional_graph) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/functional_graph.cpp) / [カテゴリ索引](README.md)
