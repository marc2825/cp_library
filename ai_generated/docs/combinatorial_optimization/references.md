# 一次資料・設計判断

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorial_optimization/index.html)

2026-10-17に確認。以下のアルゴリズム・API・問題仕様を参考に、このカテゴリ向けの実装を記述しました。外部ソースの転載や実行依存はありません。二部マッチングのみリポジトリ内3.9の実装を共用します。

| 一次資料 | 参考にした点 |
|---|---|
| [CP-Algorithms: Hungarian Algorithm](https://cp-algorithms.com/graph/hungarian-algorithm.html) | 長方形のO(n²m)、potential、slackと交互路、負費用 |
| [Boost Graph: Maximum Cardinality Matching](https://www.boost.org/doc/libs/1_85_0/libs/graph/doc/maximum_matching.html) | 一般無向グラフ、mate、増加路と最大組数の関係 |
| [CP-Algorithms: Minimum-cost flow](https://cp-algorithms.com/graph/min_cost_flow.html) | 残余逆辺、単位費用最短路、平行辺、流量依存の計算量 |
| [ACL MinCostFlow API](https://atcoder.github.io/ac-library/production/document_en/mincostflow.html) | 元辺ID・流量の復元、flowとslopeの使い分け |
| [Library Checker Assignmentの仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/assignment/task.md) | 正方行列と費用・列順列の出力 |
| [Library Checker General Matchingの仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/general_matching/task.md) | 最大数と元頂点の組、500頂点の規模 |
| [AOJ GRL_6_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_B) | 指定流量、未達なら−1の形式 |
| [3.9の参考資料](../graph_flow/references.md) | Hopcroft–Karp共通APIと検証 |

## この実装の選択

Hungarianの禁止辺はoptionalで表し、有限のINFを置きません。初期potentialは各行の最小値、型は128ビットです。最大化では広い型へ移してから符号反転し、LLONG_MINの未定義動作を避けます。AssignmentProblemは同じソルバを呼び、二つの独立したHungarian実装を保守しません。

一般最大マッチングは明示的なbase配列を用いるO(V³)のEdmonds法です。上記Boost実装のデータ構造や計算量を転載したものではありません。奇数閉路を縮約してBFSを続け、元頂点の親を辿って増加路を戻します。

最小費用流はBellman–Ford初期化＋potential付きDijkstra。負費用の扱い、同じs,tへの繰り返し、reset後の辺追記はACLと契約が異なります。全成分の負閉路を拒否し、到達不能成分の負循環を黙って無視した最小費用を返さない方針です。負費用があるのでslopeの総費用は減る場合があります。

期待値は全列の単射、頂点部分集合、全辺への整数流量割当から求めます。割当の双対条件と流量保存・容量・費用も同時に確認し、ソルバ同士の一致だけを正解判定にはしません。
