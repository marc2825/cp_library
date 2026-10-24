# 一次資料と設計判断

確認日: 2026-10-24。

| 資料 | 確認した点・採用範囲 |
|---|---|
| [Luzhiled: Top Tree](https://ei1333.github.io/library/structure/dynamic-tree/top-tree.hpp.html) | preferred pathとlight枝のsplay、頂点型rake/compress契約。静的版との違いを確認。 |
| [Luzhiled: Lazy Top Tree](https://ei1333.github.io/library/structure/dynamic-tree/lazy-top-tree.hpp.html) | spine/off-spineの作用分離、light枝の保留作用、subtreeの切り離し。共通coreの出典。 |
| [取得元の固定版](https://github.com/ei1333/library/blob/088b4c8b49f85194099305d998e7113dc3d5dd2e/structure/dynamic-tree/lazy-top-tree.hpp) / [ライセンス](https://github.com/ei1333/library/blob/088b4c8b49f85194099305d998e7113dc3d5dd2e/LICENSE) | commit `088b4c8b49f85194099305d998e7113dc3d5dd2e`、Unlicense。実コードとライセンスを取得して確認。 |
| [CP-Algorithms: Prüfer code](https://cp-algorithms.com/graph/pruefer_code.html) | 最小葉の規約・出現回数と次数の対応。こちらは明示n、n=1、不正入力検査、優先度キューのO(n log n)版を独自実装。 |
| [LC: Dynamic Tree Vertex Add Path Sum](https://github.com/yosupo06/library-checker-problems/blob/master/tree/dynamic_tree_vertex_add_path_sum/task.md) | 点更新・path query・辺差し替えの入出力仕様。 |
| [LC: Dynamic Tree Subtree Add Subtree Sum](https://github.com/yosupo06/library-checker-problems/blob/master/tree/dynamic_tree_subtree_add_subtree_sum/task.md) | `(v,p)` の向きとpを親にした部分木。rawの公開仕様を取得して確認。 |

## 取り込みと変更

[共通core](../../tree/detail/top_tree_core.hpp)は上記Lazy Top Treeをadaptしたものです。`new/delete`による所有権の曖昧さをなくすため、頂点とrakeノードをunique_ptrで所有し、削除したrakeノードはfree listで再利用します。splay前に祖先から順にlazyをpushし、非可換な作用の合成順を明示しました。公開インターフェースは0始まりの頂点番号・bool/optional・値返却・move可能/コピー不可へ変更し、非lazy Specも同じcoreで使います。元コードの大きな雛形コメントは含めません。

[Unlicense本文](../../verification/licenses/Unlicense.txt)を保持し、[台帳](../../verification/provenance.json)に取得commit・元のSHA-256・変更説明・ローカルSHA-256を登録します。

中心と根変更区間は既存の `tree/diameter.hpp`、`RootedTree`、HLDを共有する追加実装です。静的Top Treeを動的版に置換せず、それぞれの計算量と用途を維持します。独立集合・距離和のSpec、ActedMonoidアダプタ、公開ラッパーは本API向けに作成しています。

ジャッジラッパーのPROBLEMは対応する入出力形式を示すもので、オンラインACの主張ではありません。
