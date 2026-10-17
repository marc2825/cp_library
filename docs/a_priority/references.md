# 3.1〜3.6 A項目の一次資料・問題例

2026-10-17確認。既存実装を共有し、追加コードはこのリポジトリのAPIに合わせて記述しました。以下は設計を比較するための資料です。参考先のオンラインAC表示は、この追加コードのACを意味しません。

| 対象 | 一次資料・問題例 | 今回の設計判断 |
|---|---|---|
| 永続Union-Find | [sotanishy: Persistent Union Find](https://sotanishy.github.io/cp-library-cpp/data-structure/unionfind/persistent_union_find.hpp.html)、[Library Checker: Persistent Unionfind](https://judge.yosupo.jp/problem/persistent_unionfind) | 既存の経路コピー木を配列として共有。部分永続は親の更新時刻と根のサイズ履歴。完全永続は版を分岐。 |
| Rollbackによる状態復元 | [Nyaan: Rollback Union Find](https://nyaannyaan.github.io/library/data-structure/rollback-union-find.hpp.html) | Rollback Moはsnapshotトークンを利用者から受け取り、左側の一時追加を戻す。既存Rollback DSUと組合せ可能。 |
| 遅延作用 | [sotanishy: Lazy Segment Tree](https://sotanishy.github.io/cp-library-cpp/data-structure/segtree/lazy_segment_tree.hpp.html)、[ACL: Lazy Segtree](https://atcoder.github.io/ac-library/production/document_ja/lazysegtree.html)、[Range Affine Range Sum](https://judge.yosupo.jp/problem/range_affine_range_sum) | 既存LazySegtreeの作用順を保持。長さ0の単位元と、未作成区間の長さ付き初期積を区別する。 |
| 二次元セグ木 | [Nyaan: 二次元セグメント木](https://nyaannyaan.github.io/library/data-structure-2d/2d-segment-tree.hpp.html) | 格子の区間積には可換性を要求する。資料の一般的な説明に頼らず、二方向の結合が整合する条件を明記。 |
| 疎な二次元点集合 | [Nyaan: Fenwick Tree on Wavelet Matrix](https://nyaannyaan.github.io/library/data-structure-2d/fenwick-tree-on-wavelet-matrix.hpp.html)、[Point Add Rectangle Sum](https://judge.yosupo.jp/problem/point_add_rectangle_sum) | 今回はx方向のBITの各ノードへy座標とBITを持たせる別構成。更新点を事前登録し、更新順は制限しない。 |
| 順位・整数集合 | [Predecessor Problem](https://judge.yosupo.jp/problem/predecessor_problem)、[Set XOR Min](https://judge.yosupo.jp/problem/set_xor_min)、[Associative Array](https://judge.yosupo.jp/problem/associative_array) | 任意キーにはTreap/RBST、固定宇宙にはFastSet、XORにはBinaryTrie。TreapとRBSTは共通の順位APIを使う。 |
| 静的区間 | [Static RMQ](https://judge.yosupo.jp/problem/staticrmq)、[Static Range Sum](https://judge.yosupo.jp/problem/static_range_sum) | DSTは非可換モノイドも扱う。Offline RMQは最左argminを返す。 |
| 永続Queue | [Persistent Queue](https://judge.yosupo.jp/problem/persistent_queue) | 任意分岐でも操作の上界が保てる祖先ダブリング版。二スタック版の線形反転を各分岐で繰り返さない。 |
| 循環・列挙・順列 | [CSES: Gray Code](https://cses.fi/problemset/task/2205)、[Library Checker: Enumerate Quotients](https://judge.yosupo.jp/problem/enumerate_quotients)、[CSES: Permutations](https://cses.fi/problemset/task/1070) | 循環配列、floor和の区間集約、Gray順の全探索、順列の列挙順位などに利用。商は閉区間、他の区間は半開と明記。 |
| 木上Mo・区間の集合 | [Vertex Set Path Composite](https://judge.yosupo.jp/problem/vertex_set_path_composite)、[AtCoder ABC256 D: Union of Interval](https://atcoder.jp/contests/abc256/tasks/abc256_d) | 木上Moの今回の集計は頂点集合への追加/削除で、非可換な経路順の合成にはHLDを使う。RangeSetは座標の区間和集合を扱う。 |

問題例は用途の参考で、全てを今回のAPI単体で解けるという意味ではありません。専用ラッパを用意した6題は [verify/a_priority](../../verify/a_priority/) にあり、24バッチ×300操作をPythonの独立解とローカル比較します。オンライン提出は行っていません。
