# D-02〜D-10 の参考資料・設計判断

確認日: 2026-10-24。外部の実装コードは転載していません。既存の内部実装の公開化・共有化と、以下の原理に基づく実装です。ジャッジ形式の確認とオンライン AC は別です。

- [C++ 規格草案・連想コンテナの要求](https://eel.is/c++draft/associative.reqmts): D-02 は標準 multiset を共有します。heap 専用の高速化を主張せず、重複・比較順・所有権を標準コンテナに委ねます。
- [Library Checker: Double-Ended Priority Queue の問題定義](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/data_structure/double_ended_priority_queue/task.md): 初期集合、0=追加、1=最小削除、2=最大削除。ローカルのジャッジ形式テストに使用。
- [Boyer / Moore: MJRTY — A Fast Majority Vote Algorithm](https://www.cs.utexas.edu/~boyer/mjrty.pdf): 異なる票の相殺と候補の再検証。D-04 は木の標準区間へ適用し、候補の相殺演算を結合的なモノイドとして扱いません。
- [Library Checker: Point Set Range Frequency の問題定義](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/data_structure/point_set_range_frequency/task.md): 0=点代入、1=区間内の指定値の頻度。D-05 は更新値を事前に知らなくてよい segment tree of maps です。
- [CMU 15-210, Fall 2011 講義資料](https://www.cs.cmu.edu/afs/cs/academic/class/15210-f11/www/lectures/all.pdf): Leftist Heaps の rank と短い右鎖、pairwise meld による構築。D-06 は既存 [k_shortest_walks](../graph_basic/k_shortest_walks.md) の方式を汎用 pool API に抽出し、線形 build と永続 meld を分けました。
- [CP-Algorithms: Deleting from a data structure in O(T(n) log n)](https://cp-algorithms.com/data_structures/deleting_in_log_n.html): 有効期間の標準区間分解と rollback。D-08/D-09 は既存 [OfflineDynamicConnectivity](../offline_query/offline_dynamic_connectivity.md) から汎用の区間登録・走査を分離しています。

D-03 は「prefix r での最後の出現位置が l 未満なら [l,r) にない」という直接の同値性を永続木に保存します。D-07 は既存 [DynamicSegtree](../range_query/dynamic_segtree.md) の区間積を保ちながら不変ノードを共有します。SortableSegtree の内部 split は順位による分割ですが、この公開版は座標による分割であり、同じ操作とは扱いません。

D-10 は既存 DSU の union by size と [RollbackArray](../basic_utility/rollback_array.md) の履歴契約を組み合わせます。付随情報の型や集約則は callback と Data に任せ、代表元・呼出し順・失敗時の履歴・借用期間を API として定めます。

[API・検証一覧](README.md) / [統合進捗](../remaining_work.md)
