---
title: 一般グラフの最大マッチング（Blossom）
documentation_of: //combinatorial_optimization/general_matching.hpp
---
# 一般グラフの最大マッチング（Blossom）

[カテゴリ・使用条件](README.md) / [操作できる図解](../../visualizations/combinatorial_optimization/index.html#matching) / [参考資料](references.md)

`GeneralMatching(n)`へ無向辺を`add_edge(u,v)`で登録し、`maximum_matching() const`で最大**組数**のGeneralMatchingResultを得ます。奇数閉路を含む一般グラフに対応します。重み付き一般マッチングではありません。

結果は`cardinality`、長さnの`mate`（未対応−1、mate[mate[v]]=v）、`pairs`です。pairsの各辺は小さい頂点を先にし、最初の頂点の昇順で並びます。同じ最大数の中で辞書順最小の対応を保証せず、辺の登録順に依存する場合があります。

各自由頂点から交互木をBFSで探索し、二つの外側頂点を結ぶ辺で奇数閉路を発見したら、共通祖先をbaseとして縮約します。baseとparentを更新し、自由な終点へ到達したら元の頂点上で増加路を反転します。探索・縮約・復元に再帰は使いません。

空・孤立点・非連結に対応。自己ループは無視し、平行辺は解くときに線形時間で重複除去します。辺追加・再計算・コピーした結果は互いに独立。負の頂点数はinvalid_argument、範囲外の頂点はout_of_range。二部グラフなら既存のHopcroft–Karpが高速です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "combinatorial_optimization/general_matching.hpp"
int main(){
    cp::GeneralMatching graph(4);
    graph.add_edge(0,1);graph.add_edge(1,2);graph.add_edge(2,0); // Odd cycle.
    graph.add_edge(2,3);
    auto r=graph.maximum_matching();
    std::cout<<r.cardinality<<'\n';
    for(auto [a,b]:r.pairs)std::cout<<a<<' '<<b<<'\n';
}
```

[実行できる使用例](../../examples/combinatorial_optimization/general_matching.cpp)。

## 計算量・数値の条件

O(V³+E) 時間・O(V+E) 領域。nは行数、mは列数（全行を割り当てるときn≤m）、Vは頂点数、Eは登録辺数です。費用のある新実装はGCC/Clangの検査付き128ビット演算を使用します。返す総費用はlong longまたはCostに収まる必要があり、超過はoverflow_errorです。[共通仕様](README.md#共通仕様)を参照してください。

## 問題・検証

[Library Checker General Matching](https://judge.yosupo.jp/problem/general_matching)形式のローカル検証コードを用意しました。独立した全割当・頂点部分集合DP・全整数フローと比較し、境界、数値極値と復元も確認します。[検証入口](../../verify/run_combinatorial_optimization.py)と[全検証範囲](README.md#ローカル検証)を参照。オンラインACは未確認です。
