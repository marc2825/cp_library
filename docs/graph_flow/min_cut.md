---
title: 最小 s-t カットの復元
documentation_of: //graph_flow/min_cut.hpp
---

# 最小 s-t カットの復元

s と t を分離する有向辺の容量合計を最小化し、始点側の頂点と切る元辺 ID を取得します。選択を二群に分ける最適化・容量付き辺の切断・最大流の証明に使います。

[始点側・カット辺を操作する図解](../../visualizations/graph_flow/index.html#cut) / [カテゴリ索引](README.md)

## 原理

最大流まで流すと、残余ネットワークで s から t へ到達できません。到達する頂点を S、残りを T とすると、元辺 S→T は全て飽和し、元辺 T→S の流量は 0 です。流量保存則から、この境界の容量が最大流量に一致します。

**残余逆辺をカット辺として返すわけではありません。** 元の有向辺を確認し、from が S、to が T の辺だけを集めます。

## usage

```cpp
#include <cassert>
#include "graph_flow/min_cut.hpp"
int main() {
    cp::Dinic<> network(3);
    network.add_edge(0, 1, 5);
    network.add_edge(1, 2, 3);
    network.max_flow(0, 2, 1); // まだ流し切っていない
    auto cut = cp::minimum_cut(network, 0, 2); // 残りも流してから復元
    assert(cut.capacity == 3); // 累計の最大流量。今回の追加量ではない
    assert(cut.source_side[0] && cut.source_side[1] && !cut.source_side[2]);
    assert(cut.edge_ids.size() == 1 && cut.edge_ids[0] == 1);
    assert(network.max_flow(0, 2) == 0);
}
```

[動く使用例](../../examples/graph_flow/min_cut.cpp) / [実装](../../graph_flow/min_cut.hpp)

## API・条件・計算量

`minimum_cut(Dinic<Cap>& network,int s,int t)` は **network を更新**して最大流まで流し、`MinimumCutResult<Cap>` を返します。

| メンバー | 意味 |
|---|---|
| `capacity` | S→T の元辺容量合計。累計の最大流量に一致 |
| `source_side` | `vector<bool>`。頂点 v が始点側なら true |
| `edge_ids` | S→T を跨ぐ元辺 ID の昇順一覧。容量 0 の辺も含む |

- 使用条件は [Dinic](dinic.md) と同じ。s≠t、全体の最大流・容量合計が Cap に収まることが条件です。既に流した状態から使う場合も、固定済みの s,t と一致させます。
- 上限で停止した後でも使えます。関数内で残りを流します。何度も呼べます。
- 頂点の列は必要なら `source_side[v]` が true の v を列挙します。S と T はともに非空で、S は s、T は t を含みます。
- 容量 0 の元辺も分割を跨げば edge_ids に入ります。辺の費用や本数を最小化するわけではありません。
- 型の最大値分を追加しても t へ届く場合、または復元する容量合計が型に収まらない場合は `overflow_error`。**例外前に流した分は残り、巻き戻しません。** これは入力条件違反を検出する補助です。
- min_cut の選択肢が複数ある場合、この残余到達性から得る一つを返します。

復元自体は O(V+E) 時間・O(V+E) 記憶（結果を含む）。未完了なら最大流の O(V²E) 時間も必要です。

## 残余到達範囲だけ欲しい場合

`network.min_cut(s)` は流れを更新せず O(V+E) で到達範囲を返します。これは計算途中にも使えますが、t がまだ到達可能なら最小 s-t カットではありません。流量上限に達したかどうかだけでは確定できず、残余ネットワークを見る必要があります。

## 問題例・検証

- [AOJ GRL_6_A Maximum Flow](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_A)：最大流とカット容量の一致を確認できます。
- [AtCoder practice2 D Maxflow](https://atcoder.jp/contests/practice2/tasks/practice2_d)：二部最大マッチングのフローへの帰着。

小グラフの全 S/T 分割と容量を比較し、元辺の向き・ID・飽和・逆向き元辺の流量、非連結、容量 0、上限停止からの完了、極値・容量超過を確認します。オンライン AC の主張ではありません。

[参考資料](references.md)
