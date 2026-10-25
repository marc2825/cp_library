# グラフの追加項目 G-01〜G-15

2026-10-25。区間辺、被覆の復元、全点対カット、向き付け、平均閉路、次数列、暗黙・増分MST、なもり、反鎖、辺彩色、制約・負閉路、最小費用素パスを追加しました。C++17、namespace cp、0始まりです。

| ID | API・説明 | 操作できる図解 |
|---|---|---|
| G-01 | [区間辺グラフ](../graph_basic/segment_tree_graph.md) | [操作](../../visualizations/graph_extensions/index.html#segment_tree_graph) |
| G-02 | [二部最小頂点被覆・最大独立集合](../graph_flow/bipartite_vertex_cover.md) | [操作](../../visualizations/graph_extensions/index.html#bipartite_vertex_cover) |
| G-03 | [最小辺被覆](../graph_combinatorics/minimum_edge_cover.md) | [操作](../../visualizations/graph_extensions/index.html#minimum_edge_cover) |
| G-04 | [Gomory–Hu Tree](../graph_flow/gomory_hu_tree.md) | [操作](../../visualizations/graph_extensions/index.html#gomory_hu_tree) |
| G-05 | [st-Numbering・Bipolar Orientation](../graph_connectivity/st_numbering.md) | [操作](../../visualizations/graph_extensions/index.html#st_numbering) |
| G-06 | [最小平均閉路 / Karp](../graph_basic/minimum_mean_cycle.md) | [操作](../../visualizations/graph_extensions/index.html#minimum_mean_cycle) |
| G-07 | [次数列の判定と構成](../graph_combinatorics/degree_sequence.md) | [操作](../../visualizations/graph_extensions/index.html#degree_sequence) |
| G-08 | [Borůvka・暗黙MST・XOR-MST](../graph_basic/boruvka.md) | [操作](../../visualizations/graph_extensions/index.html#boruvka) |
| G-09 | [Incremental Minimum Spanning Forest](../graph_basic/incremental_msf.md) | [操作](../../visualizations/graph_extensions/index.html#incremental_msf) |
| G-10 | [無向なもり分解](../graph_connectivity/unicyclic_decomposition.md) | [操作](../../visualizations/graph_extensions/index.html#unicyclic_decomposition) |
| G-11 | [DAGの最大反鎖](../graph_combinatorics/dag_maximum_antichain.md) | [操作](../../visualizations/graph_extensions/index.html#dag_maximum_antichain) |
| G-12 | [一般単純グラフのΔ+1辺彩色](../graph_combinatorics/vizing_edge_coloring.md) | [操作](../../visualizations/graph_extensions/index.html#vizing_edge_coloring) |
| G-13 | [差分制約](../graph_basic/difference_constraints.md) | [操作](../../visualizations/graph_extensions/index.html#difference_constraints) |
| G-14 | [負閉路の抽出](../graph_basic/negative_cycle.md) | [操作](../../visualizations/graph_extensions/index.html#negative_cycle) |
| G-15 | [最小費用の辺素・頂点素パス](../graph_combinatorics/min_cost_disjoint_paths.md) | [操作](../../visualizations/graph_extensions/index.html#min_cost_disjoint_paths) |

## 検証

```sh
make verify-graph-extensions
python3 -m tools.cp_tools.suite --runners graph_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_extensions
python3 -m tools.cp_tools.suite --sanitize --runners graph_extensions
node verify/visualizations/graph_extensions_model_test.js
node verify/visualizations/graph_extensions_browser_test.mjs --browser /path/to/chromium
python3 -m tools.cp_tools.provenance
```

runnerは`--compiler`、`--sanitize`、`--links-only`対応。15公開ヘッダと拡張したLCTの単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の15使用例、図解のC++参照出力を検査します。C++比較検証はDebug/NDEBUG両方で実行します。

- n≤5の一般無向グラフ1,100個を列挙し、全辺部分集合の最小辺被覆、全頂点順列のst順序、全カットのGomory–Hu値、彩色の衝突を確認。
- n≤5の次数列、左右3以下の全二部グラフ、n≤5の全ラベル順DAGを全探索。反鎖と鎖分割の頂点被覆を確認。
- 1,800有向グラフの全単純閉路から平均・負閉路を独立計算。証拠の元辺ID・重み和・解の全不等式を確認。
- 160区間辺グラフを完全展開して全点対距離を比較。MSTは22,100個の挿入prefixをKruskalと比較し、全64bitのXOR完全グラフも照合。
- 1,000グラフ×辺素/頂点素×k=0..3の8,000条件を全単純パスの組合せと比較。零費用・逆向き辺・平行辺・復元したパスの費用と素性を確認。
- 5万頂点の長いグラフ、LCT追加・ノード再利用、4,096頂点XOR-MST、空・孤立・非連結・loop・不正入力・整数の極値を確認。
- 図解は15サンプルをC++と比較し、独立oracleを使う1,898トレース、ブラウザの15モード・入力編集・再生・不正入力表示・390px表示を検証。

G-15は非負整数費用で、指定本数の総費用を最小化します。負費用一般の単純パス最適化へは拡張しません。G-05は仮想st辺を許す仕様、G-03は一般/二部のloopなし、G-10は連結な1閉路グラフです。型・空入力・返却値の詳細は各ページを参照してください。

[一次資料・ライセンス](references.md) / [実装一覧](../../docs/remaining_work.md) / [図解のモデル範囲](../../visualizations/graph_extensions/README.md)。オンラインACは未確認です。実行結果は下の記録を参照してください。

## 実行結果

通常・libstdc++ debug・ASan/UBSan/LeakSanitizerの3構成でDebug/NDEBUGを通過しました。3記録とも完了時の同一ソースfingerprintに一致します。

| 構成 | 所要時間 | 実行記録 |
|---|---:|---|
| 通常 | 47.3秒 | [standard.json](../../verification/evidence/graph_extensions/standard.json) |
| 標準ライブラリdebug | 67.8秒 | [stdlib-debug.json](../../verification/evidence/graph_extensions/stdlib-debug.json) |
| ASan/UBSan/LeakSanitizer | 87.8秒 | [sanitize.json](../../verification/evidence/graph_extensions/sanitize.json) |

65個の高次数・密グラフの彩色、零容量/平行辺/loopを含む160個のカット木も追加確認しました。別途uint64最大値の容量・MST・平均閉路・制約とint8費用の型検証が成功しました。既存`verify/run_tree.py`もDebug/NDEBUG・7ジャッジ形式336ケース・20万頂点を通過しています。

図解モデル1,898トレース、ブラウザ15モード・4変更入力・再生・不正入力・390px表示が成功し、desktop/mobileの画像を確認しました。ChromiumとLeakSanitizerは既知のsocket/ptrace制限のため承認レビュー付きで同じローカル検証を実行しています。CIにrunnerと図解モデルを登録しましたが、GitHub Actions自体の実行とオンラインACは未確認です。
