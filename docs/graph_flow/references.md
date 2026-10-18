# 3.9 の参考資料と設計判断

2026-10-15 に以下の一次資料を参照しました。アルゴリズム・契約・問題形式を参考にし、このカテゴリの C++ / JavaScript は独自の反復実装として記述しています。外部ライブラリの実行依存やソースの転載はありません。

| 資料 | 参考にした内容 | このライブラリでの判断 |
|---|---|---|
| [ACL MaxFlow](https://atcoder.github.io/ac-library/production/document_en/maxflow.html) | 追加流量、辺 ID ごとの容量・流量、残余到達性 | max_flow は追加量を返す。s,t はリセットまで固定し、任意の辺書換えは提供しない |
| [CP-Algorithms Dinic](https://cp-algorithms.com/graph/dinic.html) | 残余辺・層・blocking flow・current arc・一般容量の上界 | 経路スタックで反復処理し、残余辺は元辺ごとに対で管理 |
| [Luzhiled Bipartite Flow](https://ei1333.github.io/library/graph/flow/bipartite-flow.hpp.html) | 左右の分割、Hopcroft–Karp の層、組の復元 | 必要な最大マッチングに絞り、毎回空から再計算。被覆や動的削除は範囲外 |
| [KACTL HopcroftKarp](https://github.com/kth-competitive-programming/kactl/blob/main/content/graph/HopcroftKarp.h) | 左右の対応、BFS と層に沿う DFS、計算量 | 再帰を用いず、最短の自由右頂点への距離を明示し、左右両方の対応を返す |
| [AOJ GRL_6_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_A) | 最大流の入力・出力 | 対応するローカル検証ラッパーを用意 |
| [Library Checker Bipartite Matching](https://judge.yosupo.jp/problem/bipartitematching) / [問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/graph/bipartitematching/task.md) | 左右の辺入力・最大数と組一覧の出力 | 相手選択を固定せず、存在する辺・頂点の一意性・最大数を検査 |
| [AOJ GRL_7_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_7_A) | 組数のみの出力 | もう一つのジャッジ形式として確認 |
| [AtCoder practice2 D](https://atcoder.jp/contests/practice2/tasks/practice2_d) | 格子の二部グラフへの帰着 | 利用例として紹介。オンライン提出結果は主張しない |

最小カットの高水準関数は、未完了の残余状態を誤って最小と解釈しないように、最大流まで流す契約としています。下位の min_cut(s) は純粋な残余到達性として残しました。容量 0 の元辺も S→T を跨げば復元し、内部の残余逆辺と区別します。

図解は C++ と同じアルゴリズムの主要な状態を記録します。JavaScript の期待値は C++ を正解として使わず、全カット列挙と部分集合 DP によって別に確認しています。

[カテゴリ索引](README.md)

3.7〜3.11 A補完の[一次資料・問題例・設計判断](../graph_a/references.md)。旧Radix Heap・SCC・Block-Cut・Dinic・BinaryLiftingは共通実装を利用しています。

3.7〜3.11のB追加分は[一次資料・設計判断](../b_7_11/references.md)を参照してください。
