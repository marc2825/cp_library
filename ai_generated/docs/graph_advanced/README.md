# グラフの残り G-16・G-20〜G-33

2026-10-25。G-16・G-20〜G-33の15項目を整備しました。G-17〜G-19の行はありません。 C++17、namespace cp、0始まり。

| ID | API・説明 | 操作できる図解 |
|---|---|---|
| G-16 | [Cost Scaling Min-Cost b-Flow](../graph_flow/cost_scaling_b_flow.md) | [操作](../../visualizations/graph_advanced/index.html#cost_scaling_b_flow) |
| G-20 | [少種類重みの最短路](../graph_basic/few_weight_shortest_path.md) | [操作](../../visualizations/graph_advanced/index.html#few_weight_shortest_path) |
| G-21 | [Shortest Nonzero Path](../graph_basic/shortest_nonzero_path.md) | [操作](../../visualizations/graph_advanced/index.html#shortest_nonzero_path) |
| G-22 | [Minimum Diameter Spanning Tree](../graph_basic/minimum_diameter_spanning_tree.md) | [操作](../../visualizations/graph_advanced/index.html#minimum_diameter_spanning_tree) |
| G-23 | [SPQR Tree（分離対探索版）](../graph_connectivity/spqr_tree.md) | [操作](../../visualizations/graph_advanced/index.html#spqr_tree) |
| G-24 | [一般グラフの木幅分解](../graph_connectivity/tree_decomposition.md) | [操作](../../visualizations/graph_advanced/index.html#tree_decomposition) |
| G-25 | [Cographの標準形](../graph_connectivity/cograph_canonical_form.md) | [操作](../../visualizations/graph_advanced/index.html#cograph_canonical_form) |
| G-26 | [一般グラフ最大重みMatching](../graph_flow/general_weighted_matching.md) | [操作](../../visualizations/graph_advanced/index.html#general_weighted_matching) |
| G-27 | [Rank-Maximal Matching](../graph_flow/rank_maximal_matching.md) | [操作](../../visualizations/graph_advanced/index.html#rank_maximal_matching) |
| G-28 | [動的単位容量Max Flow](../graph_flow/dynamic_unit_flow.md) | [操作](../../visualizations/graph_advanced/index.html#dynamic_unit_flow) |
| G-29 | [ラベル付き連結グラフ数](../graph_combinatorics/connected_graph_count.md) | [操作](../../visualizations/graph_advanced/index.html#connected_graph_count) |
| G-30 | [強連結有向グラフ数](../graph_combinatorics/strongly_connected_graph_count.md) | [操作](../../visualizations/graph_advanced/index.html#strongly_connected_graph_count) |
| G-31 | [Euler閉路数（BEST定理）](../graph_combinatorics/euler_circuit_count.md) | [操作](../../visualizations/graph_advanced/index.html#euler_circuit_count) |
| G-32 | [一般グラフの厳密同型判定](../graph_combinatorics/graph_isomorphism.md) | [操作](../../visualizations/graph_advanced/index.html#graph_isomorphism) |
| G-33 | [二端子直並列グラフ分解](../graph_combinatorics/series_parallel_decomposition.md) | [操作](../../visualizations/graph_advanced/index.html#series_parallel_decomposition) |

## 仕様を定めた項目と性能範囲

G-20はk種類の非負重みでO(m+kn)探索、G-25はcographの色付き標準形、G-28は固定端点・有向単位容量辺の動的最大流、G-33は有向二端子直並列グラフの式木です。

SPQRは分離対全探索の多項式時間版で、線形時間版ではありません。最小木幅はn<=22の厳密subset DP、別APIのmin-fillは上界のみ。一般同型は色精緻化と全探索の厳密解で、最悪指数時間です。Shortest Nonzero Pathは任意の群に対するFastSUPで単純パスを復元し、復元1回は最悪O(n²)です。rank-maximalは多倍長整数の厳密な辞書式目的を用います。

G-26のBlossomはBoost Graph 1.89の修正版を固定commitから隔離して取り込みました。システムのBoost Graphヘッダを基本型に使用しますが、古いmatching本体は使いません。[出典・ライセンス](references.md)を参照してください。

## 検証

```sh
make verify-graph-advanced
python3 -m tools.cp_tools.suite --runners graph_advanced
python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced
python3 -m tools.cp_tools.suite --sanitize --runners graph_advanced
node verify/visualizations/graph_advanced_model_test.js
node verify/visualizations/graph_advanced_browser_test.mjs --browser /path/to/chromium
python3 -m tools.cp_tools.provenance
```

15公開ヘッダとBlossom coreの単独/重複include、全カテゴリ混在2翻訳単位、説明と同一の15使用例、図解との15参照値、4比較テストをDebug/NDEBUGで検査します。

- b-flow 1,000全探索と大容量40例の既存SSP比較。流量境界・保存則・最適双対を確認。
- 一般重み付きmatching 2,500 bitmask DP比較、151頂点の密な奇数グラフ。rank-maximal 500全探索と80段階の順位。
- 動的最大流18,000更新を全カットと比較。フロー保存則とmin-cutも確認。
- 500少種類距離比較、群ラベル単純パス3,200全探索（Z/3Zと非可換S3）、最小直径木500全域木列挙。2万頂点の経路も確認。
- 5頂点以下の全単純グラフでSPQR・木幅・cographを検査。Rの三重連結、仮想辺対・分解木・実辺一意性、全消去順序とbag連結性、誘導P4を確認。多重SPQR250例、16頂点cliqueも追加。
- 一般同型350全頂点置換比較、二端子SP250辺部分集合DPと2万頂点の直列式木。
- 無向n<=6・有向n<=4の全グラフ、合成数mod、Euler辺列800全探索、自己ループ・先頭固定の階乗を確認。
- 図解モデル1,016トレース。ブラウザで全15モード、6変更入力、再生、エラー、ローカルリンク、390px表示を検証。

[図解のモデル範囲](../../visualizations/graph_advanced/README.md) / [残作業一覧](../remaining_work.md)。オンラインACとGitHub Actions上の実行は未確認です。

## 実行結果

2026-10-25、通常・標準ライブラリdebug・ASan/UBSan/LeakSanitizerの全構成でDebug/NDEBUGが成功しました。3記録は完了時の同一ソースfingerprintに一致します。

| 構成 | 所要時間 | 記録 |
|---|---:|---|
| 通常 | 57.4秒 | [standard.json](../../verification/evidence/graph_advanced/standard.json) |
| 標準ライブラリdebug | 79.6秒 | [stdlib-debug.json](../../verification/evidence/graph_advanced/stdlib-debug.json) |
| ASan/UBSan/LeakSanitizer | 104.1秒 | [sanitize.json](../../verification/evidence/graph_advanced/sanitize.json) |

前回G-01〜G-15も通常・Debug/NDEBUGで再検証（43.7秒）し、[記録](../../verification/evidence/graph_extensions/standard.json)は同じソースに一致します。図解1,016トレース、ブラウザ15モード・6変更入力・390px表示とdesktop/mobile画像を確認しました。平行辺と自己ループは曲線で描き分けます。出典13記録・明示追跡16ファイル、進捗245行（実装104、残141）、workflow構文も確認済みです。

ChromiumのsocketとLeakSanitizerのptrace制限に対応するため、同じローカル検証を承認レビュー付きで実行しました。GitHub Actions上の実行とオンライン提出ACは未確認です。
