# 3.10 の参考資料と設計判断

2026-10-16 に以下の一次資料を参照しました。アルゴリズム・API・問題形式を参考にし、今回の C++ / JavaScript はこのライブラリ向けに独自に記述しています。外部の実行依存や、ソースコードの転載はありません。

| 資料 | 参考箇所 | 今回の判断 |
|---|---|---|
| [sotanishy Triangle Enumeration](https://sotanishy.github.io/cp-library-cpp/graph/enumerate_triangles.hpp) | 次数による辺の向き、三角形の一意な列挙 | コールバックで集計・打切り。出力三頂点は数値順に整える |
| [sotanishy Clique Enumeration](https://sotanishy.github.io/cp-library-cpp/graph/enumerate_cliques.hpp) | 全クリーク列挙の目的と問題形式 | 昇順の候補の交差を使い、結果の全保持を要求しない |
| [KACTL MaximalCliques](https://github.com/kth-competitive-programming/kactl/blob/main/content/graph/MaximalCliques.h) | Bron–Kerbosch の P/X/pivot | 64ビット、P 内の隣接数が最大の pivot、昇順の頂点列、停止可能なコールバック |
| [KACTL MaximumClique](https://github.com/kth-competitive-programming/kactl/blob/main/content/graph/MaximumClique.h) | 彩色によるクリークサイズの上界 | 動的な次数ヒューリスティック等は入れず、独立した単純な彩色上界の分枝限定法を記述 |
| [sotanishy Chromatic Number](https://sotanishy.github.io/cp-library-cpp/graph/chromatic_number.hpp) | 彩色数と部分集合の扱い | 今回は O(3^N) の厳密 DP を選択。最小頂点を含む独立集合の分割で、彩色も復元する |
| [CP-Algorithms Kirchhoff theorem](https://cp-algorithms.com/graph/kirchhoff-theorem.html) | ラプラシアンの余因子と全域木数 | 素数体上の行列式。多重辺・ループ・負の整数重み・全余因子を検証 |

KACTL の MaximalCliques は CC0、MaximumClique はファイル内に GPL3+ の表記があることを確認しました。いずれもコードを取り込まず、アルゴリズムの考え方を参照しています。最大クリーク実装には参照先のコードや適応的な次数順の処理を含めていません。

入力・出力は次の問題定義を参照しました。

- [Library Checker Enumerate Triangles](https://judge.yosupo.jp/problem/enumerate_triangles) / [定義](https://github.com/yosupo06/library-checker-problems/blob/master/graph/enumerate_triangles/task.md)
- [Library Checker Enumerate Cliques](https://judge.yosupo.jp/problem/enumerate_cliques) / [定義](https://github.com/yosupo06/library-checker-problems/blob/master/graph/enumerate_cliques/task.md)
- [Library Checker Maximum Independent Set](https://judge.yosupo.jp/problem/maximum_independent_set) / [定義](https://github.com/yosupo06/library-checker-problems/blob/master/graph/maximum_independent_set/task.md)
- [Library Checker Chromatic Number](https://judge.yosupo.jp/problem/chromatic_number) / [定義](https://github.com/yosupo06/library-checker-problems/blob/master/graph/chromatic_number/task.md)
- [Library Checker Counting Spanning Tree (Undirected)](https://judge.yosupo.jp/problem/counting_spanning_tree_undirected) / [定義](https://github.com/yosupo06/library-checker-problems/blob/master/graph/counting_spanning_tree_undirected/task.md)
- [AtCoder ABC002 D 派閥](https://atcoder.jp/contests/abc002/tasks/abc002_4)

検証の期待値は、C++ の結果を JavaScript の正解にする方法ではなく、全頂点／辺部分集合、直接の色割当、全順列による行列式から別に求めます。オンライン提出や全てのジャッジ最大ケースでの時間保証は主張していません。

[カテゴリ索引](README.md)

3.7〜3.11のB追加分は[一次資料・設計判断](../b_7_11/references.md)を参照してください。
