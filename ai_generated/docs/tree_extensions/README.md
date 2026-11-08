# 木の追加項目 T-01〜T-03・T-20〜T-23

木の追加APIとして7項目を提供します。木の中心、Prüfer符号、HLDの根変更補助を加え、動的4項目は共通のrake/compress基盤で辺の追加・削除まで扱います。

| ID | API・説明 | 図解 |
|---|---|---|
| T-01 | [木の中心](../tree/tree_center.md) | [操作](../../visualizations/tree_extensions/index.html#tree_center) |
| T-02 | [Prüfer Code](../tree/prufer_code.md) | [操作](../../visualizations/tree_extensions/index.html#prufer_code) |
| T-03 | [根変更時の部分木](../tree/rerooted_subtree.md) | [操作](../../visualizations/tree_extensions/index.html#rerooted_subtree) |
| T-20 | [Dynamic Top Tree](../tree/dynamic_top_tree.md) | [操作](../../visualizations/tree_extensions/index.html#dynamic_top_tree) |
| T-21 | [Lazy Top Tree](../tree/lazy_top_tree.md) | [操作](../../visualizations/tree_extensions/index.html#lazy_top_tree) |
| T-22 | [動的部分木集約](../tree/dynamic_subtree_aggregate.md) | [操作](../../visualizations/tree_extensions/index.html#dynamic_subtree_aggregate) |
| T-23 | [Fully Dynamic Tree DP](../tree/fully_dynamic_tree_dp.md) | [操作](../../visualizations/tree_extensions/index.html#fully_dynamic_tree_dp) |

```sh
make verify-tree-extensions
python3 verify/run_tree_extensions.py --sanitize
python3 -m tools.cp_tools.suite --stdlib-debug --runners tree_extensions
node verify/visualizations/tree_extensions_model_test.js
node verify/visualizations/tree_extensions_browser_test.mjs --browser /path/to/chromium
python3 -m tools.cp_tools.provenance
```

`--compiler`、`--links-only`にも対応。単独・重複includeをDebug/NDEBUGで検査し、全カテゴリ混在の2翻訳単位、本文と一致する7使用例、図解の参照出力を確認します。C++単体検証はDebug/NDEBUGの双方で実行します。

- Prüferはn=2..7の全符号、相異なる辺集合、独立した最小葉走査と比較。
- 中心は840木の全頂点からの距離、整数の半整数位置・零重み・巨大整数・小数・不正入力を比較。
- 根変更区間はn=1..55の森の全根・全頂点をBFSと比較。20万頂点の鎖も確認。
- 動的木は15万混合操作で、代入・符号反転・加算の順序、パスと枝の作用範囲、link/cut後の値、非連結・失敗操作、moveを比較。10万頂点の鎖・星型、ノード再利用も確認。
- Tree DPは15000段階の更新で独立集合の全列挙と距離和BFS、非可換な文字列compressを比較。
- Library Checkerの2形式を、各12本の独立Python生成入力（辺の差し替え・作用・問い合わせ）で照合。
- 図解は入力編集・前後移動・再生・不正入力表示・390px表示を確認。各操作の意味とrake/compress分解を見せるモデルであり、C++のsplay回転列は再現しません。

ローカル検証とオンラインACは区別します。オンライン提出は未実施です。[出典・ライセンス](references.md) / [木カテゴリ](../tree/README.md) / [全図解](../../visualizations/index.html)。
