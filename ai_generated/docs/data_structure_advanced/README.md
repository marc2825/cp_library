# データ構造 D-20〜D-35

[前回のD-02〜D-10](../data_structure_extensions/README.md) / [出典・設計判断](references.md) / [16モードの図解](../../visualizations/data_structure_advanced/index.html)。

## 実装一覧

| ID | 公開APIと説明 | 実装の範囲 |
|---|---|---|
| D-20 | [非可換群DSU](../union_find/group_potentialized_disjoint_set_union.md) | 群の順序を保持する差分制約、矛盾判定 |
| D-21 | [Fully Dynamic Connectivity](../union_find/fully_dynamic_connectivity.md) | ETTの全域森・小成分の代替辺走査、多重辺・自己ループ |
| D-22 | [Kinetic Segment Tree](../range_query/kinetic_segtree.md) | 整数時刻の線形値、certificate・点変更・区間最小 |
| D-23 | [RBST Segment Tree](../ordered_structure/rbst_segment_tree.md) | 動的列の点更新・区間積・反転・split/concat |
| D-24 | [Division Beats](../range_query/division_segment_tree_beats.md) | 非負整数のfloor除算・加算・代入・sum/min/max |
| D-25 | [Static Range Mode](../static_query/static_range_mode.md) | 平方分割候補と頻度再検証、同率の順序を定義 |
| D-26 | [Static Range LIS](../static_query/static_range_lis.md) | Seaweed / unit-Monge、重複値・汎用Compare |
| D-27 | [Static Range Inversions](../offline_query/static_range_inversions.md) | Mo + Fenwickを公開関数に集約 |
| D-28 | [Dynamic Wavelet Matrix](../range_query/dynamic_wavelet_matrix.md) | uint64の挿入・削除・点変更、rank/select・順序統計 |
| D-29 | [Dynamic Bit Vector](../ordered_structure/dynamic_bit_vector.md) | 挿入・削除・rank/select、ノードを再利用 |
| D-30 | [Finger Tree](../ordered_structure/finger_tree.md) | 計測付き永続2–3木、両端・split/concat |
| D-31 | [Retroactive Priority Queue](../ordered_structure/partially_retroactive_priority_queue.md) | bridge法、登録時刻で過去操作を編集、更新O(log m) |
| D-32 | [Persistent Lazy RBST](../ordered_structure/persistent_lazy_rbst.md) | 完全永続の列、monoid作用・反転、過去版の分岐 |
| D-33 | [直交範囲クエリ](../offline_query/orthogonal_range_query.md) | 1〜10次元、静的点集合の重み和と個数 |
| D-34 | [Dynamic Nearest Neighbor](../range_query/dynamic_nearest_neighbor.md) | 1〜10次元、追加・削除・移動と最近傍 |
| D-35 | [Multidimensional Fenwick](../range_query/fenwick_tree_nd.md) | 疎な登録座標、点加算・prefix/box和 |

D-23・D-24・D-29・D-34の曖昧だった範囲は表の操作として仕様化しました。D-21はHDTの多項対数時間保証を持たず、小さい成分を走査します。D-29は圧縮構造ではなく O(n) words、D-30はstrict C++のため両端操作も最悪 O(log n) としています。D-34はKD-treeと更新bufferの再構築方式で、問い合わせの最悪計算量は線形です。個別説明で計算量と適用範囲を確認できます。

## 検証

```bash
make verify-data-structure-advanced
python3 -m tools.cp_tools.suite --runners data_structure_advanced
python3 -m tools.cp_tools.suite --stdlib-debug --runners data_structure_advanced
python3 -m tools.cp_tools.suite --sanitize --runners data_structure_advanced
node verify/visualizations/data_structure_advanced_model_test.js
node verify/visualizations/data_structure_advanced_browser_test.mjs --browser /path/to/chromium
```

- 16ヘッダの単独・重複includeをDebug/Releaseで確認し、全カテゴリ混在の2翻訳単位をリンクします。
- 16件の文書内usageと実ファイルを一致確認し、実行結果を図解モデルの参照値とも照合します。
- [query_test.cpp](../../verify/data_structure_advanced/query_test.cpp): 4要素置換群、一般グラフのBFS比較、多重辺・自己ループ、長さ7までの3値全列・全区間でLIS/Mode/Inversions、各種block幅、LISの2万要素（stdlib-debugのみ2千）、1/2/3/5次元の全点走査、Beatsの配列更新、Kineticの全線形値比較。
- [sequence_test.cpp](../../verify/data_structure_advanced/sequence_test.cpp): 7,000回の永続版分岐とaffine作用、非可換文字列の反転、Finger Treeの10,000操作と3万要素、rank/selectの2万操作、bit幅1/7/32/64のWavelet Matrixをソート・配列と比較。16bitの5千要素の回転更新も確認します。
- [retroactive_spatial_test.cpp](../../verify/data_structure_advanced/retroactive_spatial_test.cpp): 5万登録挿入＋5万delete-min（stdlib-debugのみ各千）、2比較順で各5万回の過去編集とmultisetの全履歴再生、長さ7までの全有効履歴に対する操作削除、1/2/3/5/10次元の3,000更新と全点距離比較。
- Static Range LIS / Static Range Inversionsの2ジャッジ形式を、各12入力の独立Python計算と比較します。
- 図解モデルは3,016比較、ブラウザは全16モード・5変更入力・再生・入力エラー・ローカルリンク・390px表示を確認します。
