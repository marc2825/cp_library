# 3.11 木アルゴリズム

今回の S 項目を既存の `tree/` にまとめます。共有の [Graph](../../graph/graph.hpp) を使い、C++17・`namespace cp`・0始まり・半開区間が基本です。必要なヘッダだけ、または `tree/all.hpp` を include します。

| 解きたいこと | 実装・API | 説明 | 図解 |
|---|---|---|---|
| 親・深さ・部分木サイズ・DFS順 | `rooted_tree.hpp` / `RootedTree<T>` | [説明](rooted_tree.md) | [図解](../../visualizations/tree/index.html#rooted) |
| LCA・祖先・距離・パス上の移動 | `doubling_lca.hpp` / `DoublingLCA<T>` | [説明](doubling_lca.md) | [図解](../../visualizations/tree/index.html#lca) |
| Euler Tour・部分木区間 | `euler_tour.hpp` / `EulerTour<T>` | [説明](euler_tour.md) | [図解](../../visualizations/tree/index.html#euler) |
| 木の直径・復元 | `diameter.hpp` / `tree_diameter` | [説明](diameter.md) | [図解](../../visualizations/tree/index.html#diameter) |
| 一意な遷移を k 回適用 | `binary_lifting.hpp` / `BinaryLifting` | [説明](binary_lifting.md) | [図解](../../visualizations/tree/index.html#lifting) |
| 子から親への基本の木DP | `tree_dp.hpp` / `tree_dp` | [説明](tree_dp.md) | [図解](../../visualizations/tree/index.html#dp) |
| 全ての頂点を根とした木DP | `rerooting.hpp` / `rerooting` | [説明](rerooting.md) | [図解](../../visualizations/tree/index.html#rerooting) |

既存の LCA・RootedTree・EulerTour・直径の API を保持し、RootedTree に帰りがけの `postorder` を追加しました。HLD・重心・重心分解は引き続き [従来のガイド](../graph_tree.md) を参照してください。旧 `graph_theory/Tree/tree_template.hpp` も保持します。

## 共通条件と選び方

木のAPIは無向の木・森を入力し、自己ループ・多重辺を含む閉路を拒否します。根付きの処理は指定した根の成分を先に、それ以外は最小番号の頂点を根にします。空も構築できます。距離の算術条件は [RootedTree](rooted_tree.md) に記載。直径だけは非負重み・連結が条件です。

BinaryLifting は Graph を使わず遷移先の配列を受け取り、閉路・自己ループを許容します。`-1` は終端です。LCA が必要なら DoublingLCA、祖先や任意の遷移の反復だけなら BinaryLifting を選びます。

木DPのコールバックは `merge(S,S)->S`、`add_vertex(S,v)->R`、`lift(R,送信元→受信先の辺)->S`。S と R は別の型にできます。全方位木DPでは結合則と単位元が必要です。親側も含めて隣接順を保つので、非可換の演算にも対応します。状態の演算が重い場合はその費用を計算量に掛けます。

部分木クエリでは preorder 上の `[tin,tout)` を [Fenwick](../range_query/fenwick_tree.md) や [Segtree](../range_query/segtree.md) へ渡します。RMQ の LCA を実装したい場合は EulerTour.walk と [Sparse Table](../static_query/sparse_table.md) を組み合わせられます。

## 検証

```sh
python3 verify/run_tree.py
python3 verify/run_tree.py --sanitize
python3 verify/run_graph_tree.py --sanitize --stress
node verify/visualizations/tree_model_test.js
node verify/visualizations/tree_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++` 等でコンパイラを変更できます。C++17・警告エラー、10ヘッダの単独 include、既存カテゴリ・旧APIとの混在、複数翻訳単位、使用例7本、7ジャッジ形式・336ケースを確認します。

単体検証は339個の非空な小さな森を全根で、300個のランダムな森／木を全頂点対で確認します。再帰DFS・BFSパス・全頂点部分集合・全根からのDPと比較し、文字列連結による非可換集約と辺の送受信方向も確認。終端を含む遷移表700個、uint64の境界、20万頂点のパス・星型・閉路を含みます。空・不正な木・距離超過も検査します。オンライン AC は未確認です。

図解は1720バッチ・34238スナップショットを独立オラクルと比較します。全方位DPの各有向メッセージ・prefix/suffix、部分木区間、往復列、全DP状態、巨大なkの閉路も検査します。ブラウザの操作・入力検査・再生・リンク・390px表示は別のブラウザ検証で確認します。

[一次資料と設計判断](references.md) / [動く使用例](../../examples/tree/) / [全図解](../../visualizations/index.html)

## 高度なアルゴリズムの追加

- [Virtual Tree](virtual_tree.md) / [操作できる図解](../../visualizations/advanced/index.html#virtual)
- [DSU on Tree](dsu_on_tree.md) / [操作できる図解](../../visualizations/advanced/index.html#sack)
- [Link-Cut Tree](link_cut_tree.md) / [操作できる図解](../../visualizations/advanced/index.html#lct)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

## A項目の補完（2026-10-17）

| 機能 | 実装 | 説明・使用例 | 図解 |
|---|---|---|---|
| 木のハッシュ | [hpp](../../tree/tree_hash.hpp) | [説明](tree_hash.md) | [操作](../../visualizations/graph_a/index.html#tree_hash) |

今回の追加/共有入口は `python3 verify/run_graph_a.py` / `--sanitize`（`make verify-graph-a`）で検証します。[検証範囲・共有設計](../graph_a/README.md) / [一次資料と問題例](../graph_a/references.md)。オンラインACは未確認です。
