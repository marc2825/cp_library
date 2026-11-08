---
title: Dinic 最大流
documentation_of: //graph_flow/dinic.hpp
---

# Dinic 最大流

有向辺の容量を超えず、中間頂点で流入と流出を一致させて、s から t へ送れる量を最大化します。物資の輸送・容量付き割当・辺を切る最適化等で使います。

[層・増加路・逆辺を操作する図解](../../visualizations/graph_flow/index.html#dinic) / [カテゴリ索引](README.md)

## 原理

元辺 u→v の容量 c、流量 f に対し、正向きには c−f、逆向きには f 流せます。逆辺は過去の選択を取り消し、別の経路へ回すために使います。

1. 残余容量が正の辺で BFS し、始点からの最短辺数 level を求める。
2. `level[to]=level[v]+1` を満たす辺だけで増加路を探す。
3. 層の s→t 経路が全て詰まるまで流し、BFS へ戻る。
4. 残余辺で t へ届かなければ最大流が確定する。

各頂点の current arc は次に調べる隣接辺の位置です。使えない辺を同じ段階で繰り返し走査しません。DFS は頂点列・辺列・経路の最小残余容量を明示的に持ちます。

## usage

```cpp
#include <cassert>
#include "graph_flow/dinic.hpp"
int main() {
    cp::Dinic<long long> network(3);
    int id = network.add_edge(0, 1, 5);
    network.add_edge(1, 2, 3);
    assert(network.max_flow(0, 2, 1) == 1); // 今回追加した量
    assert(network.max_flow(0, 2) == 2);    // 累計は 3
    assert(network.max_flow(0, 2) == 0);
    auto edge = network.get_edge(id);
    assert(edge.capacity == 5 && edge.flow == 3);
    network.reset_flow();
    assert(network.get_edge(id).flow == 0);
    assert(network.max_flow(2, 0) == 0);    // リセット後は s,t を変更可
}
```

[動く使用例](../../examples/graph_flow/dinic.cpp) / [実装](../../graph_flow/dinic.hpp)

## API と条件

| API | 意味 | 時間 |
|---|---|---|
| `Dinic<Cap>(n=0)` | n 頂点、辺なし | O(V) |
| `size()` / `edge_count()` | 頂点数 / 元辺数 | O(1) |
| `add_edge(u,v,capacity)` | 有向の元辺を追加し、0 始まりの登録 ID を返す | ならし O(1) |
| `get_edge(id)` | `{from,to,capacity,flow}` の現在値 | O(1) |
| `edges()` | 元辺を ID 順で取得 | O(E) |
| `max_flow(s,t,limit=max(Cap))` | 残余状態から高々 limit 追加して、その追加量を返す | 一般に O(V²E)、E=0 は O(V) |
| `min_cut(s)` | 現在の残余辺で s から到達できるかの `vector<bool>` | O(V+E) |
| `reset_flow()` | 容量・辺 ID を保ち、全流量を 0 にする。s,t の固定も解除 | O(E) |

- `Cap` は bool 以外の整数型です。容量は非負。浮動小数点の誤差判定は扱いません。
- n≥0、頂点・辺 ID は範囲内。max_flow には s≠t、limit≥0 が必要です。元辺数は残余辺 ID が int に収まる範囲です。
- 最初の max_flow から reset_flow まで、同じ s,t を使います。limit=0 でも固定します。`minimum_cut` も同じ条件です。
- 各辺容量と全体の最大流量が Cap に収まる入力を使います。省略 limit は型の最大値であり、それを超える真の答えを自動で表せません。小さい上限で停止していても、問題全体の答えが収まる型を推奨します。
- 自己ループには流れを送らず、平行辺・逆向きの元辺は別 ID として扱います。逆辺を利用した分は元辺の flow が減ります。
- フロー後の辺追加も可能です。新しい元辺の流量は 0。既存の流れを保持して同じ s,t で続行できます。容量の変更・任意の流量書換えは提供しません。
- `edges()` と `get_edge()` は値を返し、以降の操作に追従しません。

本体の記憶 O(V+E)、max_flow の追加作業領域 O(V)。V²E の上界は一回の呼出しについてです。繰り返すたびに探索用配列と BFS を作り直します。

## 注意点・問題例

容量と費用は異なります。この実装は流す量を最大化し、費用の最小化は行いません。無向容量辺を表すなら、元辺を両向きに登録します。自動追加の逆辺に容量を持たせる操作とは異なります。

- [AOJ GRL_6_A Maximum Flow](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_A)：[検証プログラム](../../verify/graph_flow/max_flow.test.cpp)。
- [AtCoder practice2 D Maxflow](https://atcoder.jp/contests/practice2/tasks/practice2_d)：二部グラフへの帰着、採用辺の復元。

全カット列挙との比較、流量保存・容量制約・残余辺・繰り返し・辺追加・リセット・符号なし型・型最大値・20 万頂点の鎖を検証します。範囲は [カテゴリの検証](README.md#使用例と検証) を参照。

[最小カットの復元](min_cut.md) / [参考資料](references.md)
