---
title: グラフ：フロー・マッチング・最適化
documentation_of: //graph_flow/all.hpp
---

# 3.9 グラフ：フロー・マッチング・最適化

整数容量の最大流と最小 s-t カット、二部グラフの最大マッチングを扱います。C++17、`namespace cp`、0 始まりです。まとめて使う入口は `graph_flow/all.hpp`。標準ライブラリだけで動作します。

| やりたいこと | 説明・usage | 実装 | 操作できる図解 |
|---|---|---|---|
| 容量制約の下で、始点から終点へ最大限流す | [Dinic](dinic.md) | [dinic.hpp](../../graph_flow/dinic.hpp) | [層・増加路・逆辺](../../visualizations/graph_flow/index.html#dinic) |
| 始点側の頂点と、切る元辺を復元する | [最小カット](min_cut.md) | [min_cut.hpp](../../graph_flow/min_cut.hpp) | [到達範囲・S→T の辺](../../visualizations/graph_flow/index.html#cut) |
| 左右から、一頂点一組で最大数の組を作る | [二部最大マッチング](bipartite_matching.md) | [bipartite_matching.hpp](../../graph_flow/bipartite_matching.hpp) | [交互路と組み替え](../../visualizations/graph_flow/index.html#matching) |

[3.23 組合せ最適化](../combinatorial_optimization/README.md)には、一般グラフのBlossom最大マッチング、Hungarian／Assignment、最小費用流を追加しています。二部版はこのカテゴリと同じ実装を共有します。

## 選び方・共通の条件

- Dinic は容量と残余辺を内部管理する独立したクラスです。3.7 / 3.8 の `Graph` とは別に構築します。容量は距離・コストと異なり、非負の整数です。
- `Dinic<Cap>` は `bool` 以外の整数型。通常は `int` / `long long` を使います。各辺容量と問題全体の最大流量・カット容量を表せる型を選びます。
- フローは有向辺を登録し、残余逆辺を自動で作ります。平行辺・逆向きの元辺・自己ループ・容量 0・非連結に対応します。
- `max_flow(s,t,limit)` の戻り値は**今回追加した量**です。残余状態を保持します。同じ s,t で繰り返せますが、始点・終点を変える前には `reset_flow()` が必要です。上限 0 の最初の呼出しでも s,t を固定します。
- `min_cut(s)` は現在の残余到達範囲だけを返します。上限付き最大流の直後には、最小カットとは限りません。`minimum_cut(network,s,t)` は残りを流してから容量・頂点側・元辺 ID を復元します。
- 二部最大マッチングは左右の分割を入力で指定します。左右それぞれ 0 始まり。3.8 の [二部グラフ判定](../graph_connectivity/bipartite.md) と用途が異なります。最大数を保証し、辞書順の最小性は保証しません。
- 向き・頂点番号・非負条件・s≠t は assert による使用条件です。`NDEBUG` 時は利用者が条件を守ります。返された配列は値のスナップショットです。
- 空の Dinic は構築できますが、フローには異なる二頂点が必要です。マッチングは片側が 0 でも利用できます。

どちらの探索も反復実装です。深い経路に対する再帰は使いません。既存 API の変更はありません。

## 計算量

V は頂点数、E は登録した元辺数です。マッチングでは V=L+R です。

| 操作 | 時間 | 記憶 |
|---|---|---|
| Dinic 構築 / 辺追加 | O(V) / ならし O(1) | O(V+E) |
| 一般容量の最大流 | O(V²E)、E=0 なら O(V) | 追加作業 O(V)、本体 O(V+E) |
| 最大流後のカット復元 | O(V+E) | O(V+E)（結果を含む） |
| Hopcroft–Karp | O((V+E)√(V+1)) | O(V+E) |

最小カットの高水準関数は必要なら最大流の時間もかかります。上限を小さく指定して何度も max_flow を呼ぶと、それぞれ BFS 等をやり直します。

## 使用例と検証

- [制限付き追加流量・辺の流量](../../examples/graph_flow/dinic.cpp)
- [最小カットの頂点側・元辺 ID](../../examples/graph_flow/min_cut.cpp)
- [最大マッチングの組一覧](../../examples/graph_flow/bipartite_matching.cpp)

```sh
python3 verify/run_graph_flow.py
python3 verify/run_graph_flow.py --sanitize
node verify/visualizations/graph_flow_model_test.js
node verify/visualizations/graph_flow_browser_test.mjs --browser /path/to/chromium
```

4 ヘッダの単独 include（debug/release）、逆順 include、他カテゴリとの混在、複数翻訳単位、3 使用例、3 ジャッジ形式・192 ケースを確認します。C++ は 738 容量割当と 689 二部グラフの全探索、700 組のランダムケースを独立した全カット列挙・部分集合 DP と比較します。流量保存則、残余到達性、各辺の上限、上限停止／再開、リセット、容量型の極値も検証します。20 万頂点のフロー経路と 20 万左頂点の長い交互路を含みます。**オンライン AC 済みという意味ではありません。**

[参考資料・設計判断](references.md) / [図解一覧](../../visualizations/index.html) / [引き継ぎ書](../../HANDOFF.md)

## A項目の補完（2026-10-17）

| 機能 | 実装 | 説明・使用例 | 図解 |
|---|---|---|---|
| 下限制約付きフロー | [hpp](../../graph_flow/lower_bound_flow.hpp) | [説明](lower_bound_flow.md) | [操作](../../visualizations/graph_a/index.html#lower_bound_flow) |
| Project Selection Problem | [hpp](../../graph_flow/project_selection.hpp) | [説明](project_selection.md) | [操作](../../visualizations/graph_a/index.html#project_selection) |

今回の追加/共有入口は `python3 verify/run_graph_a.py` / `--sanitize`（`make verify-graph-a`）で検証します。[検証範囲・共有設計](../graph_a/README.md) / [一次資料と問題例](../graph_a/references.md)。オンラインACは未確認です。
