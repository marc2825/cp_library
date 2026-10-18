# 3.1〜3.6 Bの一次資料・設計判断

アルゴリズムの説明・不変条件を参照し、このリポジトリのC++17・namespace cp・半開区間のAPIに合わせて実装しています。公開コードの丸ごとの取り込みは行っていません。既存Radixなどを共有する場合、その実体の出典を保持します。

<a id="search"></a>
## 整数・分数・探索

- [CP-Algorithms: Stern–Brocot Tree](https://cp-algorithms.com/others/stern_brocot_tree_farey_sequences.html): mediantとEuclidによる連続移動。分数経路はrun単位、境界探索は分子・分母の両方に上限を設けます。
- 整数k乗根は浮動小数点近似を補正する方法を避け、乗算前の除算で判定します。非単調探索は利用者が与える区間下界の分枝限定として定義し、無条件の少数評価を保証しません。

<a id="dsu"></a>
## Union-Findと橋

- [CP-Algorithms: Finding Bridges Online](https://cp-algorithms.com/graph/bridge-searching-online.html): 二つのDSUと縮約森、small-to-largeの根変更、閉路の圧縮。実装は反復圧縮、64bit訪問世代、多重辺・自己ループの検証を追加しています。
- [ACL: DSU](https://github.com/atcoder/ac-library/blob/master/atcoder/dsu.hpp): 通常DSUの契約比較。Parallelは長さ2^kの区間同士の併合を階層化し、既に処理した併合を飛ばします。スレッド並列ではありません。
- Offline Dynamic ConnectivityとRangeSetは既存実装を共有し、DSUの成分加算では頂点固有値と成分tagを分けます。

<a id="range"></a>
## 区間・静的クエリ

- [Luzhiled: Segment Tree Fractional Cascading](https://ei1333.github.io/library/structure/segment-tree/segment-tree-fractional-cascading.hpp): 根での二分探索と子prefixへの対応。今回の木はstable merge時のprefix個数を直接保持します。連鎖catalogのFractionalCascadingも別APIで提供します。
- [CP-Algorithms: Sqrt Tree](https://cp-algorithms.com/data_structures/sqrt-tree.html): prefix・suffix・between、最高相違bitによる層選択。今回は静的版に絞り、更新用の内部Sqrt Treeは追加していません。
- [ACL: Lazy Segtree](https://github.com/atcoder/ac-library/blob/master/atcoder/lazysegtree.hpp): 作用の合成順とモノイドの条件。Persistent Lazyはcarry検索、2D Dualは可換作用として公開します。
- [Luzhiled: Wavelet Matrix](https://ei1333.github.io/library/structure/wavelet/wavelet-matrix.hpp): bitごとのstable partition。重み和版は各partitionのprefixを持ち、同値keyのk選択を元の添字順にします。
- [出題者のRange Sort構造の設計](https://github.com/yosupo06/library-checker-problems/issues/818): sorted runの値域木split/meldと外側の区間積。点のkey・payload更新も提供します。
- [Library Checker問題一覧](https://github.com/yosupo06/library-checker-problems): staticrmq、point_set_range_sort_range_composite、rectangle_sum、static_range_count_distinctなどの入出力・用途の確認。オンラインACとは区別します。

<a id="ordered"></a>
## 平衡木・heap

- [Sedgewick/Wayne: RedBlackBST](https://github.com/kevin-wayne/algs4/blob/master/src/main/java/edu/princeton/cs/algs4/RedBlackBST.java): 左傾赤黒木の色・回転・move-redの不変条件。今回はC++の添字所有・多重集合・部分木サイズとして実装し、全操作列で黒高さを検査します。
- [Sleator/Tarjan: Self-Adjusting Binary Search Trees](https://www.cs.cmu.edu/~sleator/papers/self-adjusting.pdf): Splayの自己調整と償却評価。今回はkey多重集合として提供します。
- Weight-Balanced Treeは特定の回転定数に依存せず、3/4の節点数条件を破った部分木を再構築します。説明の更新上界は保守的に償却O(log² N)とします。永続版は既存mutable木をコピーせず、不変Treap節点を共有します。
- [iwiwi: Radix Heap](https://github.com/iwiwi/radix-heap/blob/master/README_ja.md): 単調keyのbucket分配。既存 `data_structure/radix_heap_iwiwi.hpp` を共有し、新しい入口は符号なし整数だけに制限します。
- [Luzhiled: Priority Sum Structure](https://ei1333.github.io/luzhiled/snippets/structure/priority-sum-structure.html): 上位k個の分割。今回のcp版は二つのmultisetで不在削除も扱います。

<a id="spatial"></a>
## 二次元・オフライン

- [maspypy: Rectangle Add Rectangle Sum](https://maspypy.github.io/library/ds/offline_query/rectangle_add_rectangle_sum.hpp.html): 二次元差分と係数queryの考え方。今回は更新矩形の四隅を登録した疎なBITでオンラインに処理し、境界の±1は使いません。
- [maspypy: KdTree Monoid](https://maspypy.github.io/library/ds/kdtree/kdtree_monoid.hpp): 閉bounding boxと半開矩形の使い分け。今回は静的な報告／最近点と、オンライン挿入・lazy affine・部分再構築の別APIを用意します。
- [Princeton: Kd-Trees](https://www.cs.princeton.edu/courses/archive/spring19/cos226/demos/99DemoKdTree.pdf): 空間の分割と矩形・最近点の枝刈り。最近点の距離はlong doubleの近似値であることを明記します。
- Range TreeはxのSegtreeと各節点のyのFenwick Treeを組み合わせ、登録点IDの更新を扱います。Hilbert順はMoの操作契約を保ち、全格子の連続性を検証します。種類数は右端走査と最後の出現のBITです。

[追加APIと検証](README.md) / [図解のモデル範囲](../../visualizations/b_1_6/README.md)
