# 3.7〜3.11 A項目の一次資料・問題例

2026-10-17確認。追加コードは既存のGraph・距離の算術補助・Radix Heap・SCC・二重連結分解・Dinic・BinaryLiftingを共有し、各APIと検証をこのリポジトリ向けに整備しています。

| 対象 | 一次資料・問題例 | 設計判断 |
|---|---|---|
| Radix Dijkstra | [iwiwi/radix-heap](https://github.com/iwiwi/radix-heap/blob/master/README_ja.md)、[AOJ GRL_1_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_A) | 旧ヒープを共有。距離は符号なしキーで格納し、親と元辺IDを返す。 |
| Dial・DAG | [Shortest Path問題](https://judge.yosupo.jp/problem/shortest_path) | DialはC+1個の循環バケット。DAGは閉路を拒否してトポロジカル順に負辺も緩和。 |
| Johnson | [AOJ GRL_1_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_C) | 全点を初期値0とするBellman–Ford、整数reweight、元辺IDを保った復元。 |
| 単純なK-shortest | [Luzhiled: K Shortest Path](https://ei1333.github.io/library/graph/shortest-path/k-shortest-path.hpp.html) | Yen法。頂点を重複させず、多重辺をIDで区別。接頭辞trieで禁止辺を共有し全経路列挙と比較。 |
| K-shortest walk | [Luzhiled: K Shortest Walk](https://ei1333.github.io/library/graph/shortest-path/k-shortest-walk.hpp.html)、[Library Checker](https://judge.yosupo.jp/problem/k_shortest_walk) | Eppstein法。整数indexの永続leftist heapとsidetrack履歴でメモリを所有し、walkは遅延復元。 |
| Functional Graph | [Nyaan: Functional Graph](https://nyaannyaan.github.io/library/graph/functional-graph.hpp.html)、[CSES Planets Queries II](https://cses.fi/problemset/task/1160) | 入次数の剥がしで閉路と流入木を分解。64bit jumpは既存部品、最小遷移回数も公開。 |
| Block-Cut | [既存の二重連結分解](../../graph/biconnected_components.hpp)、[Biconnected Components](https://judge.yosupo.jp/problem/biconnected_components) | 元頂点を全て保持する所属関係の森。既存公開APIを再実装しない。 |
| 2-SAT | [ACL公式仕様](https://atcoder.github.io/ac-library/production/document_ja/twosat.html)、[Library Checker](https://judge.yosupo.jp/problem/two_sat) | 節の真偽値指定を明確にし、既存の反復SCCから割当を復元。 |
| Eulerian Trail | [Library Checker有向版](https://judge.yosupo.jp/problem/eulerian_trail_directed)、[無向版](https://judge.yosupo.jp/problem/eulerian_trail_undirected) | Hierholzer法を反復実装。論理辺ID、自己ループ、離れた辺成分、指定始点を検証。 |
| 下限/選択フロー | [ACL公式MaxFlow仕様](https://atcoder.github.io/ac-library/production/document_ja/maxflow.html)、[AtCoder ABC193 F](https://atcoder.jp/contests/abc193/tasks/abc193_f) | Dinicを共有。下限は頂点需要へ変換し、s-tの負のvalueも許容。選択問題は費用/罰金/必須条件を分離し、soft総和+1で必須条件を扱う。 |
| 木ハッシュ | [Nyaan: Tree Hash](https://nyaannyaan.github.io/library/tree/tree-hash.hpp.html)、[Rooted Tree Isomorphism Classification](https://judge.yosupo.jp/problem/rooted_tree_isomorphism_classification) | 高さごとのsaltと子の積、部分木サイズ、二重mod。非根付きは重心から計算。反復走査と固定seed、衝突の限界を明記。 |

参考実装のAC表示をこの追加コードのACとして扱いません。6形式のローカル検証は[verify/graph_a](../../verify/graph_a/)に置き、非一意の割当/道は出力を解析して条件そのものを確認します。
