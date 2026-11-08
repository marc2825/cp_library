# 高度なアルゴリズムの追加

[カテゴリ一覧](../README.md) / [操作できる解説](../../visualizations/advanced/index.html) / [一次資料と設計判断](references.md)

指定された20項目を確認し、既存実装のあるものは再実装せず、不足分を11カテゴリ・21ヘッダとして追加しました。新しい機能は各カテゴリの `all.hpp` から利用できます。

| 依頼項目 | 実装先 | 対応 |
|---|---|---|
| Segment Tree Beats | [range_query](../range_query/segment_tree_beats.md) | 追加 |
| Wavelet Matrix | [static_query](../static_query/wavelet_matrix.md) | 追加 |
| Rollback DSU＋Offline Dynamic Connectivity | [union_find](../union_find/rollback_disjoint_set_union.md) / [offline_query](../offline_query/offline_dynamic_connectivity.md) | 追加 |
| Persistent Segment Tree | [履歴木](../ordered_structure/persistent_segment_tree.md) / [区間k番目](../ordered_structure/persistent_order_statistics.md) | 追加 |
| Implicit Treap / RBST | [ordered_structure](../ordered_structure/implicit_treap.md) | Implicit Treapを追加 |
| Li Chao Tree / CHT | [Li Chao](../dp_optimization/li_chao_tree.md) / [既存CHT](../../data_structure/ConvexHullTrick.hpp) | 旧CHTを保持、Li Chaoを追加 |
| Mo with Updates | [offline_query](../offline_query/mo_with_updates.md) | 追加 |
| Heavy-Light Decomposition | [tree/heavy_light_decomposition.hpp](../../tree/heavy_light_decomposition.hpp) / [既存ガイド](../graph_tree.md) | 実装済み、保持 |
| Virtual Tree | [tree](../tree/virtual_tree.md) | 追加 |
| DSU on Tree | [tree](../tree/dsu_on_tree.md) | 追加 |
| Link-Cut Tree | [tree](../tree/link_cut_tree.md) | 追加 |
| SMAWK / Monotone Minima | [SMAWK](../dp_optimization/smawk.md) / [Monotone Minima](../dp_optimization/monotone_minima.md) | 両方追加 |
| Min-Plus Convolution | [dp_optimization](../dp_optimization/min_plus_convolution.md) | 両凸・片凸を追加 |
| Miller–Rabin＋Pollard’s Rho | [素数判定](../number_theory_convolution/miller_rabin.md) / [素因数分解](../number_theory_convolution/pollard_rho.md) | 高度な数論カテゴリへ追加 |
| Subset Convolution | [set_function](../set_function/subset_convolution.md) | 追加 |
| FFT/NTT＋任意mod畳み込み | [既存FFT/NTT](../convolution/README.md) / [任意mod](../convolution/convolution_mod.md) | FFT/NTTを保持、任意modを追加 |
| FPS inv/log/exp/pow/sqrt | [fps](../fps/README.md) | 実装済み、保持 |
| Bostan–Mori＋Berlekamp–Massey | [linear_recurrence](../linear_recurrence/README.md) | 実装済み、保持 |
| Aho–Corasick＋Eertree | [AC](../string_algorithm/aho_corasick.md) / [回文木](../string_algorithm/eertree.md) | 両方追加 |
| 一般グラフマッチング | [combinatorial_optimization](../combinatorial_optimization/general_matching.md) | Blossom実装済み、保持 |

旧 `data_structure/segtree_beats.hpp`、`data_structure/DynamicConectivity.hpp`、`graph_theory/Tree/AuxiliraryTree.hpp` は空の予約ファイルだったため、新ヘッダへの互換include入口にしました。空ファイルに公開APIはありませんでした。

## 検証

```sh
python3 verify/run_advanced.py
python3 verify/run_advanced.py --sanitize
make verify-advanced
node verify/visualizations/advanced_model_test.js
node verify/visualizations/advanced_browser_test.mjs --browser /path/to/chromium
```

追加のPython/npmパッケージは不要。`--compiler clang++` でコンパイラを変更できます。ヘッダ単独／重複includeのdebug/release、カテゴリall.hppの混在・逆順・複数翻訳単位、21使用例とMarkdown usageの一致、ジャッジ形式のサンプル、独立した素朴解との比較を実行します。オンラインACの確認はしていません。

- 構造：105,000回のBeats操作、37,500回の静的順序統計質問、6,000回の分岐した永続更新、30,000回の非可換Treap操作、48,000回の多重辺イベントとBFS、更新付きMoを初期化し直して3種類の幅で比較。
- 木：135,000回の動的木操作を独立BFSパスと比較。Virtual Treeの全頂点対LCA閉包・最寄り祖先、DSU on Treeの部分木集合を比較。Link-Cut／Virtual Tree／DSU on Treeで20万頂点の鎖を確認。
- 最適化・数論・畳み込み：凸／片凸の二重ループ、Monge行列の全行探索、SMAWKの評価回数、最左同点、Li Chaoの全直線／線分走査、10万未満の素数篩、強擬素数、uint64上限・大きい半素数の正確な分解。任意modの二重ループ・64bit超のCRT係数、全submaskによるSubset Convolutionと比較。
- 文字列：長さ9までの全二値列と任意byte列で回文を全列挙。ACは空・重複・再build・追記・NUL／FFを直接比較し、20万文字の深いTrie／回文木も検証。

図解は小さい入力の学習用モデルです。C++実装の性能や全内部ポインタ配置を再現するものではなく、[簡略化範囲](../../visualizations/advanced/README.md)を明記しています。
