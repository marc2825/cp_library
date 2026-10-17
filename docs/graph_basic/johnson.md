---
title: Johnson's Algorithm
documentation_of: //graph_basic/johnson.hpp
---

# Johnson's Algorithm

負辺を含む疎グラフで全点対最短路と復元情報を取得。

## API・使用条件

johnson(Graph<T>) -> optional<vector<ShortestPathResult<T>>>。行sのsource=s。

Tは符号付き整数。有向/無向可、全体のどこかに負閉路があればnullopt。空は成功した空列。potential、reweightの加減算、各Dijkstraと元距離への復元の途中値が有限範囲内。超過overflow_error。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/johnson.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,-2); g.add_edge(1,2,3); g.add_edge(0,2,5);
    auto all=cp::johnson(g); std::cout<<(*all)[0].dist[2]<<" "<<(*all)[2].reachable(0)<<"\n";
}
```

出力: `1 0`。

## 原理・計算量

全頂点に距離0を与えたBellman–Fordでpotential hを求め、w+h[u]-h[v]>=0へ変換する。各始点からDijkstraし、d=d'-h[s]+h[v]で元に戻す。辺追加順を保つため復元IDも元グラフと一致。

O(NM+N(N+M)log(N+M+1))時間、O(N²+M)領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#johnson) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/johnson.cpp) / [カテゴリ索引](README.md)
