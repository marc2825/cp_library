# DP・探索の追加 P全14項目

P-01〜P-10・P-20〜P-23を整備しました。P-11〜P-19の行はありません。C++17、namespace cp、添字は0始まり（SATのliteralのみ±1始まり）。

| ID | API・説明 | 入力を操作できる図解 |
|---|---|---|
| P-01 | [Frontier DP・連結性の境界状態](../dp_optimization/frontier_dp.md) | [操作](../../visualizations/dp_extensions/index.html#frontier_dp) |
| P-02 | [Bit-parallel LCS](../dp_optimization/bit_parallel_lcs.md) | [操作](../../visualizations/dp_extensions/index.html#bit_parallel_lcs) |
| P-03 | [Myers Bit-parallel Edit Distance](../dp_optimization/myers_edit_distance.md) | [操作](../../visualizations/dp_extensions/index.html#myers_edit_distance) |
| P-04 | [Exact Cover・Algorithm X・Dancing Links](../dp_optimization/exact_cover.md) | [操作](../../visualizations/dp_extensions/index.html#exact_cover) |
| P-05 | [A* 状態空間探索](../dp_optimization/a_star.md) | [操作](../../visualizations/dp_extensions/index.html#a_star) |
| P-06 | [IDA* 反復閾値探索](../dp_optimization/ida_star.md) | [操作](../../visualizations/dp_extensions/index.html#ida_star) |
| P-07 | [双方向状態空間BFS](../dp_optimization/bidirectional_search.md) | [操作](../../visualizations/dp_extensions/index.html#bidirectional_search) |
| P-08 | [汎用CNF SAT Solver（DPLL）](../dp_optimization/sat_solver.md) | [操作](../../visualizations/dp_extensions/index.html#sat_solver) |
| P-09 | [ナップサック・Bitset部分和の復元](../dp_optimization/knapsack_reconstruction.md) | [操作](../../visualizations/dp_extensions/index.html#knapsack_reconstruction) |
| P-10 | [最適解の個数を返すDP](../dp_optimization/optimal_solution_count.md) | [操作](../../visualizations/dp_extensions/index.html#optimal_solution_count) |
| P-20 | [長方形Monge行列のmin-plus積](../dp_optimization/monge_matrix_product.md) | [操作](../../visualizations/dp_extensions/index.html#monge_matrix_product) |
| P-21 | [重み付きSlope Trick](../dp_optimization/weighted_slope_trick.md) | [操作](../../visualizations/dp_extensions/index.html#weighted_slope_trick) |
| P-22 | [遷移を更新できる有限状態DP](../dp_optimization/dynamic_finite_state_dp.md) | [操作](../../visualizations/dp_extensions/index.html#dynamic_finite_state_dp) |
| P-23 | [重さの種類が少ない高速ナップサック](../dp_optimization/small_weight_knapsack.md) | [操作](../../visualizations/dp_extensions/index.html#small_weight_knapsack) |

## 仕様と共通基盤

P-10はLISの添字列・LCSの一致添字pair列・0-1 Knapsackの品物ID集合を数えます。P-20は有限値の長方形Monge min-plus積、P-21は重み付きhingeとmerge/反転、P-22は固定状態数の遷移行列列の更新、P-23は正の重量の種類が少ない0-1 Knapsackです。

既存のBigInt、SMAWK、KnapsackResultと品物型、DP安全演算を共有します。汎用SATはwatched literalを持つDPLLで最悪指数時間。Frontierは幅に指数的、Exact CoverとIDA*も最悪指数時間です。各APIの前提・領域量・最適性条件を個別説明に記載しました。[一次資料・設計判断](references.md)。

## 検証

```sh
make verify-dp-extensions
python3 -m tools.cp_tools.suite --runners dp_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners dp_extensions
python3 -m tools.cp_tools.suite --sanitize --runners dp_extensions
node verify/visualizations/dp_extensions_model_test.js
node verify/visualizations/dp_extensions_browser_test.mjs --browser /path/to/chromium
```

14公開ヘッダと探索共通型、単独/重複include、全カテゴリ混在2翻訳単位、14件の説明と一致する使用例、図解参照値を検査します。4比較テストをDebug/NDEBUGの両方で実行します。

- 長さ6までの全二進列の組、複数wordランダム1500組、63/64/65等の境界、10万文字。通常LCS・編集距離と比較。
- 最適解数800例の添字列全探索、ナップサック1500例の部分集合/個数列と復元、少種類重量300例の通常DP比較、10万品物と部分和20万。
- A*/IDA*の600グラフをFloydと比較。非整合hの再展開、ゼロ費用閉路、双方向600有向グラフ、明示stack深さ2万。
- Frontierはn<=5の1099単純グラフ・200多重グラフの全辺部分集合、1000頂点の梯子。Exact Coverは600行部分集合全探索・8-Queens=92・callback例外後の再利用。
- SATは仮定を含む5000回の真理値表比較、鳩の巣原理UNSAT、2万単位節と節追加後の再求解。
- Monge600長方形積を三重ループと比較し、最左argminと評価回数を確認。重み付きSlope Trick4800段階を単位版と整数点走査で比較、10^30の重み・符号極値・自己merge。
- 動的DP6000更新/区間問い合わせを逐次DPと比較。状態列復元、空区間、負費用、更新overflow時の元状態保持。

[図解のモデル範囲](../../visualizations/dp_extensions/README.md) / [残作業一覧](../remaining_work.md)。オンラインACとGitHub Actionsでの実行は未確認です。

## 実行結果

2026-10-26、通常・標準ライブラリdebug・ASan/UBSan/LeakSanitizerの全構成でDebug/NDEBUGを通過しました。3件の記録は同じ完了時ソースfingerprintに一致します。

| 構成 | 所要時間 | 記録 |
|---|---:|---|
| 通常 | 55.2秒 | [standard.json](../../verification/evidence/dp_extensions/standard.json) |
| 標準ライブラリdebug | 79.4秒 | [stdlib-debug.json](../../verification/evidence/dp_extensions/stdlib-debug.json) |
| ASan/UBSan/LeakSanitizer | 94.1秒 | [sanitize.json](../../verification/evidence/dp_extensions/sanitize.json) |

既存の基本DPも[通常38.5秒](../../verification/evidence/dp_optimization/standard.json)、既存Monge/Slope等を含むB-12〜15も[通常53.0秒](../../verification/evidence/b_12_15/standard.json)で回帰検証が成功しました。この2件も同じソースに一致します。

図解モデルは1,494トレースとC++使用例14件、ブラウザは全14モード・7変更入力・再生/戻る/エラー・ローカルリンク・幅390pxを検証。Frontier・Myers・重み付きSlope Trickの画像も確認しました。Chromiumはsandboxのsocket制限で起動できなかったため、同じローカル検証を承認レビュー付きで実行。LeakSanitizerも既知のptrace制限に従って承認レビュー付きで実行しました。

出典台帳13レコード・16明示追跡ファイル、680ヘッダ・335 C++テスト・50runnerの検証台帳、進捗245行（実装118、残127）、workflow構文を確認しました。オンライン提出AC・GitHub Actions上の実行は未確認です。既存の他カテゴリの実行記録は当時のソースに対する履歴で、今回の追加後は保守的fingerprintにより失効します。
