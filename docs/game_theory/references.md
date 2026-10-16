# 一次資料・設計判断

[カテゴリ索引](README.md) / [図解](../../visualizations/game_theory/index.html)

2026-10-17に確認。アルゴリズム・ルール・問題形式を参考に、このカテゴリ向けに実装を記述しました。外部ソースの転載や実行依存はありません。

| 一次資料 | 参考にした点 |
|---|---|
| [CP-Algorithms: Sprague–Grundy theorem. Nim](https://cp-algorithms.com/game_theory/sprague-grundy-nim.html) | 通常NimのXOR、misèreの終盤、mexと独立和の条件 |
| [CSES Nim Game I](https://cses.fi/problemset/task/1730/) | 最後を取ると勝ち、複数ケース、first/second出力 |
| [HackerRank Misère Nim](https://www.hackerrank.com/challenges/misere-nim-1/problem) | 最後を取ると負け、First/Secondの形式 |
| [CSES Nim Game II](https://cses.fi/problemset/task/1098/) | 一手で1..3個取り、周期4のGrundy値を使う例 |
| [AtCoder DP K - Stones](https://atcoder.jp/contests/dp/tasks/dp_k) | 除去可能数の集合から有限DAGを作る例、10万状態 |
| [既存トポロジカルソート](../graph_basic/topological_sort.md) | 既存Graphの有向辺と順序の規約 |

## 設計判断

Nimの山は符号付き入力も検査し、内部のXORと復元値にはuint64_tを使います。石数の総和を計算せず、最大値を含む入力の加算超過を避けます。Misèreの空局面は手番側の勝ちとして定義し、合法な勝ち手の有無とは分けました。

Grundy計算は隣接リストと既存Graphの両方へ対応。全成分をKahn法で検査し、逆順にmexを計算します。重複辺でも入次数が正しく減るようsize_tを使い、mexの印は頂点番号で再利用して全頂点での初期化を避けます。20万状態の深い経路に再帰を使いません。

DAGGameはグラフとGrundy表を一緒に保持し、外から表だけを変更できないAPIにしました。独立和の勝ち手は、一成分のGrundy値を小さくする辺を一つ復元します。通常ルールの定理を一般のmisèreへ拡張するAPIは設けません。

検証はXOR判定同士の一致だけにせず、小さなNimの全合法手、DAG成分の積ゲーム、集合によるmexを別に計算します。大きな山はPythonの任意精度整数とも比較します。ジャッジ形式のローカル検証はオンラインACとは区別します。
