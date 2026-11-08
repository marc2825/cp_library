# グラフ・木ライブラリの既存の検証履歴と参考資料

機能分離前のコメントを保存したものです。現行コードの検証は `verify/` を参照してください。

## graph_theory/graph_template.hpp

```text
★ Name        ： Graph/Edge template
 ■ Snipet      : No

 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description : （ここに含まれてるデアを書く）-> oj bundleを活用して分割してもいいかも


 ■ Usage       :


 ■ Verify      :・次数確認  : https://atcoder.jp/contests/abc061/submissions/44696570 （出次数）
                             https://atcoder.jp/contests/arc037/submissions/44702792 （入次数）
                             https://atcoder.jp/contests/abc292/submissions/44706628
                             https://atcoder.jp/contests/abc288/submissions/44706798
                ・dijkstra : https://atcoder.jp/contests/abc035/submissions/44674674
                              https://atcoder.jp/contests/arc064/submissions/44674896
                              https://judge.yosupo.jp/submission/156318 (パス中の頂点の復元)
                              https://atcoder.jp/contests/abc252/submissions/44674993 (最短路木の辺集合)
                              https://atcoder.jp/contests/abc305/submissions/44706388
                              https://atcoder.jp/contests/typical90/submissions/51426032
                 ・Bellman-Ford : https://atcoder.jp/contests/abc137/submissions/44680861 (負閉路検出含む)
                                  https://atcoder.jp/contests/abc061/submissions/44680972
                 ・Floyd-Warshall : https://atcoder.jp/contests/abc051/submissions/44685373 （最短路の辺集合の復元）
                                    https://atcoder.jp/contests/abc143/submissions/44694953
                                    https://atcoder.jp/contests/abc079/submissions/44695727 (隣接行列の入力)
                            (TODO :)    https://atcoder.jp/contests/abc191/tasks/abc191_e
                                        https://atcoder.jp/contests/abc160/tasks/abc160_d
                                        https://atcoder.jp/contests/abc073/tasks/abc073_d
                                        https://atcoder.jp/contests/arc159/tasks/arc159_a
                                        https://atcoder.jp/contests/abc243/tasks/abc243_e
                                        https://atcoder.jp/contests/arc035/tasks/arc035_c
                                        https://atcoder.jp/contests/abc012/tasks/abc012_4
                 ・全頂点対最短路    : https://atcoder.jp/contests/abc016/submissions/44696629
                                      https://atcoder.jp/contests/arc025/submissions/46654773
                 ・BFS              : https://atcoder.jp/contests/abc309/submissions/44695442
                                      https://atcoder.jp/contests/abc251/submissions/44695547 (BFS木)
                                      https://atcoder.jp/contests/abc132/submissions/44702688
                                      https://atcoder.jp/contests/abc292/submissions/44706495
                                      https://atcoder.jp/contests/abc218/submissions/45759654
                                      https://atcoder.jp/contests/arc011/submissions/46668040
                                      https://atcoder.jp/contests/abc142/submissions/54725039
                                      https://atcoder.jp/contests/abc233/submissions/55200987 (Eid, prev 復元)
                 ・DFS              : https://atcoder.jp/contests/abc251/submissions/44695547 (DFS木)
                 ・0-1BFS           : https://atcoder.jp/contests/abc184/submissions/44695926
                 ・二部グラフ        : https://atcoder.jp/contests/abc282/submissions/44696396
                                      https://atcoder.jp/contests/typical90/submissions/51471838
                 ・連結成分に分解    : https://atcoder.jp/contests/abc282/submissions/44696396
                                    https://atcoder.jp/contests/arc037/submissions/44702792
                                    https://atcoder.jp/contests/abc292/submissions/44706628
                                    https://atcoder.jp/contests/abc288/submissions/44706798
                 ・Kruskal          : https://atcoder.jp/contests/abc282/submissions/44700772
                 ・Prim             : https://atcoder.jp/contests/abc282/submissions/44700838
                 ・トポロジカルソート : https://atcoder.jp/contests/dp/submissions/44703075
                                      https://codeforces.com/contest/1851/submission/220685825
                 ・サイクル検出      : https://atcoder.jp/contests/past202107-open/submissions/44703572 (有向)
                                      https://judge.yosupo.jp/submission/156511 (有向、長さ、辺集合、順序関係)
                                      https://judge.yosupo.jp/submission/156512 (無向、長さ、頂点集合、辺集合、順序関係)
                                      https://atcoder.jp/contests/abc311/submissions/44707019 (functional graph)
                 ・橋               : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9368892#1
                                      https://atcoder.jp/contests/abc075/submissions/54922838
                                      https://codeforces.com/contest/1986/submission/267284311
                 ・関節点           : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9368902#1
                 ・二重辺連結成分分解: https://judge.yosupo.jp/submission/217039
                                     https://atcoder.jp/contests/arc039/submissions/54927935
                                     https://codeforces.com/contest/1000/submission/267313257
                 ・二重（頂点）連結成分分解: https://judge.yosupo.jp/submission/217255
                 ・Block Cut Tree   : https://atcoder.jp/contests/abc318/submissions/54961604
                                      https://atcoder.jp/contests/abc334/submissions/54961918
                                    　https://atcoder.jp/contests/nadafes2022_day2/submissions/54974497 (関節点)
                　・LowLink         : https://atcoder.jp/contests/abc334/submissions/54977970

                    TODO : https://atcoder.jp/contests/abc296/tasks/abc296_e
                           https://atcoder.jp/contests/abc065/tasks/arc076_b
                           https://atcoder.jp/contests/abc277/tasks/abc277_e
                           https://atcoder.jp/contests/abc291/tasks/abc291_f
                           https://atcoder.jp/contests/iroha2019-day2/tasks/iroha2019_day2_d
                           https://atcoder.jp/contests/arc029/tasks/arc029_3
                           https://atcoder.jp/contests/abc237/tasks/abc237_e
                           鉄則本
                           yosupojudge
                           AOJ


 ■ References  : https://ei1333.github.io/library/graph/graph-template.hpp
                 https://github.com/tsuchinoko29/nok0/blob/main/graph/graph.hpp
                 https://github.com/magurofly/cp-library-rs
                 https://nyaannyaan.github.io/library/graph/graph-template.hpp
                 https://kopricky.github.io/code.html
                 https://github.com/kth-competitive-programming/kactl

 ■ TODO        : bfs等も組み込む、グリッドグラフ用、木用の構造体Treeも別でつくる、メモリが大きすぎないかの確認
```

## graph_theory/Tree/tree_template.hpp

```text
★ Name        ：木に関するアルゴリズム詰め合わせ
 ■ Snipet      : No

 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description : （ここに含まれてるデアを書く）-> oj bundleを活用して分割してもいいかも


 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/tessoku-book/submissions/44897237 (部分木の大きさ)
                 https://atcoder.jp/contests/editor-update-test/submissions/44912089 (部分木の大きさ)
                 https://atcoder.jp/contests/abc070/submissions/44904708 (重み付き深さ)
                 https://atcoder.jp/contests/abc070/submissions/44904821 (重み付き距離)
                 https://judge.yosupo.jp/submission/157557 (LCA)
                 https://atcoder.jp/contests/abc014/submissions/44909147 (重みなし距離)
                 https://atcoder.jp/contests/abc267/submissions/44920179 (重み無し直径、根の変更、Level Ancestor Problem(kth_ancestor))
                 https://judge.yosupo.jp/submission/157556 (重み付き直径、パス上の頂点列挙)
                 https://atcoder.jp/contests/abc291/submissions/44911963 (重心分解)
                 https://atcoder.jp/contests/typical90/submissions/44905006 (オイラーツアー、重み無し距離)
                 https://atcoder.jp/contests/abc294/submissions/44918371 (オイラーツアー（辺、頂点）、辺情報、重み付き、パスクエリ)
                 https://atcoder.jp/contests/abc294/submissions/44919946 (HL分解、辺情報、重み付き、パスクエリ)
                 https://atcoder.jp/contests/arc039/submissions/54927935 (is_on_path, 二重辺連結成分分解後の木)
                 https://codeforces.com/contest/1000/submission/267313257 (直径、二重辺連結成分分解後の木)
                 https://atcoder.jp/contests/abc318/submissions/54961604 (is_on_path, Block Cut Tree上)

        TODO　 : https://judge.yosupo.jp/problem/jump_on_tree
                 https://judge.yosupo.jp/problem/frequency_table_of_tree_distance



 ■ References  : https://nyaannyaan.github.io/library/tree/tree-query.hpp
                 https://hcpc-hokudai.github.io/archive/graph_tree_001.pdf
                 https://maspypy.com/euler-tour-%E3%81%AE%E3%81%8A%E5%8B%89%E5%BC%B7
                 https://qiita.com/drken/items/4b4c3f1824339b090202
                 https://kopricky.github.io/code/GraphDecomposition/centroid_decomposition.html

 ■ TODO        : 重み付き木に対応させる（Graph, Edge 構造体を有効活用する）
                 オイラーツアーやHLD分解や重心分解に典型クエリは組み込んでしまうか、別の構造体にまとめるか、
                 初期化処理をコンストラクタに組み込むかどうか（graph構造体とのすり合わせ関連）
                 easiest_HLDの実装（こどふぉのやつ）を組み込むか
                 depth[]と直接アクセスしてるのをdepth()と変えるか？
                 無向グラフの場合にEに両方向分追加するか？（e.idxとの辻褄合わせの問題）
                 オイラーツアーの辺のidx,辺と頂点とのすり合わせはこのままでいいか
                 HL分解で辺に情報をもたせる場合をもっと洗練できないか
```

## graph_theory/dijkstra.hpp

```text
★ Name        ： Dijkstra (by priority queue)
 ■ Snipet      : Yes

 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :


 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc035/submissions/44850919
 ■ References  : https://ei1333.github.io/library/graph/shortest-path/dijkstra.hpp

 ■ TODO        : graph_template とどう共存させるか（あっちは部分問題での超典型処理を瞬殺するためのもので柔軟性は微妙なので、スニペット的でいいかも）
```

## graph_theory/bfs.hpp

```text
★ Name        ： Breadth First Search (幅優先探索)　 基本形
 ■ Snipet      : Yes

 ■ Arguments   : Graph(V,E) (vector<vector<int> >), start point (int)
 ■ Return      : distances from start (vector<int>)
 ■ Complexity  :

 ■ Description :


 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc309/submissions/44418398
 ■ References  :

 ■ TODO        :
```

## graph_theory/grid_bfs.hpp

```text
★ Name         : Grid BFS
 ■ Snipet      :

 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :


 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/abc317/submissions/44957185
 ■ References  :

 ■ TODO        :
```
