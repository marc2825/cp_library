# 3.25 探索・ヒューリスティック

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

S項目を[heuristic_search/all.hpp](../../heuristic_search/all.hpp)へまとめています。固定seedと反復上限で再現でき、[Timer](timer.md)をstopへ接続して実時間でも打ち切れます。

| 説明・usage | 主な計算量 | 図解 |
|---|---|---|
| [乱数生成器](random.md) | raw/real O(1)、整数範囲は期待O(1)、shuffle期待O(n) | [操作](../../visualizations/heuristic_search/index.html#random) |
| [Timer](timer.md) | 各操作O(1) | [操作](../../visualizations/heuristic_search/index.html#timer) |
| [Simulated Annealing](simulated_annealing.md) | I回の提案・評価と状態コピー | [操作](../../visualizations/heuristic_search/index.html#annealing) |
| [Beam Search](beam_search.md) | 各深さの候補数Cに対して通常O(C log C) | [操作](../../visualizations/heuristic_search/index.html#beam) |
| [局所探索](local_search.md) | I回の近傍生成・評価、最大B近傍ならO(IB)回の評価 | [操作](../../visualizations/heuristic_search/index.html#local) |

[E全8項目のAPI・使用例・検証](../search_parsing_extensions/README.md)を追加しました。

- [汎用Undo Manager・スコア検算](../search_parsing_extensions/undo_manager.md)
- [IndexSet・一様ランダム選択](../search_parsing_extensions/index_set.md)
- [TSPの2-opt・Or-opt](../search_parsing_extensions/tsp_neighborhood.md)
- [SA受理判定の対数テーブル](../search_parsing_extensions/fast_annealing_acceptance.md)
- [差分更新型Beam Search](../search_parsing_extensions/delta_beam_search.md)
- [Chokudai Search](../search_parsing_extensions/chokudai_search.md)
- [多点並列探索・並列Multi-Start SA](../search_parsing_extensions/parallel_search.md)
- [Alias法・重み付き抽出・連続分布](../search_parsing_extensions/distribution_sampling.md)

## 共通仕様

- C++17、namespace cp、各ヘッダは単独include可能。外部依存・共有する大域乱数状態はありません。添字は0始まり。
- 探索のStateはコピー・ムーブ可能で、コピーした状態を独立に変更できる必要があります。デフォルト構築や比較演算子は不要。ポインタ等で可変データを共有する場合は利用側でコピーの独立性を保ってください。
- evaluate(const State&)はbool以外の64ビット以下の算術型を返す純粋関数。float/doubleは有限値のみ。整数は型のまま比較し、SAの悪化幅も整数で差を取ってから変換します。評価関数内のオーバーフローや制約違反は利用側で防ぎます。
- SearchGoal::minimizeが既定。maximizeに切替可能。初期解を必ず一度評価し、上限0ならそのまま返します。同点の最良解は先に得た状態を保持します。
- SearchOptionsはmax_iterations=1000、goal、任意のstd::function<bool()> stop。SA/局所探索のOptionsはこれを継承します。stopはtrueで中断。Beamはwidth=10、max_depth=100を独立に指定します。
- SearchResultはbest_state/current_state、best_score/current_score、iterations、evaluations、accepted、terminationを保持。評価回数は初期解を含み、採用数にはSAの同点も含みます。局所探索の反復数には改善なしの完了した走査も含みます。
- SearchTerminationはiteration_limit、depth_limit、local_optimum、exhausted、stopped。局所最適・候補枯渇・予算終了を区別します。最適解の証明や保証は返しません。
- stopは協調的に確認します。コールバック・評価・コピー・ソートの途中には割り込めません。初期評価はstopより先です。時間制限は厳密な実行時間上限ではありません。
- 不正な範囲・モード・温度・非有限評価値はinvalid_argument、評価回数のsize_t超過はoverflow_error。NDEBUGでも検査します。コールバックや確保の例外はそのまま伝播し、乱数・時計・利用側の副作用は巻き戻しません。
- 状態の合法性・近傍の完全性は利用側が保証します。SAは制約違反の候補を自動修復せず、既存beam_searchは同一状態の統合をせず、追加のbeam_search_uniqueがkeyで統合します。探索の品質は目的関数、近傍、温度、幅、予算に依存します。

## 検証

```sh
python3 verify/run_heuristic_search.py
python3 verify/run_heuristic_search.py --sanitize
node verify/visualizations/heuristic_search_model_test.js
node verify/visualizations/heuristic_search_browser_test.mjs --browser /path/to/chromium
```

7ヘッダの単独・重複includeと不正型拒否、他カテゴリとの逆順include・複数翻訳単位、5使用例を検証。C++では局所探索24,060ケース、Beam 2,000ケース、SA 800ケースを独立シミュレーションと比較します。Pythonでは任意精度64ビット乱数・範囲と3探索の1,550ケースを照合。debug/release、ASan/UBSan対応。時計は待機せず模擬Clockで境界を検査し、20万回の改善、幅200・深さ200、64ビット極値も扱います。

ヒューリスティックの解品質は固定出力のAC判定と異なるため、オンラインジャッジ用PROBLEMを仮置きせず、ローカルの決定的プロトコルで検証します。オンライン提出・ACやスコア改善は未確認です。図解は64ビット乱数、模擬時計、左右近傍を使う3探索の5モードです。

通常・ASan/UBSan/LeakSanitizerのdebug/release、全24カテゴリの図解モデル、5モードのブラウザ検証が成功。図解モデルは26,367入力・110,981スナップショット・25拒否条件を独立解と比較します。ブラウザは採否・期限・同点と重複・候補削減・局所最適・再生・入力拒否、SVGの描画範囲と390px幅を確認しています。

## 3.21〜3.25 A項目の補完

- [Zobrist Hash](zobrist_hash.md) / [操作できる図解](../../visualizations/final_a/index.html#zobrist)
- [Tabu Search](tabu_search.md) / [操作できる図解](../../visualizations/final_a/index.html#tabu)
- [Beam Searchの重複除去](beam_search_unique.md) / [操作できる図解](../../visualizations/final_a/index.html#beam_unique)
- [スコア差分更新とapply/undo](incremental_score.md) / [操作できる図解](../../visualizations/final_a/index.html#incremental)

[追加APIの条件・検証一覧](../final_a/README.md)。`python3 verify/run_final_a.py` / `--sanitize` / `make verify-final-a`。

## 3.21〜3.25 B項目

- [Multi-Start SA](multi_start_sa.md) / [操作できる図解](../../visualizations/b_21_25/index.html#multi_start_sa)
- [温度制御・採用率の較正](temperature_control.md) / [操作できる図解](../../visualizations/b_21_25/index.html#temperature_control)
- [パラメータチューニング](parameter_tuning.md) / [操作できる図解](../../visualizations/b_21_25/index.html#parameter_tuning)
- [多腕バンディット](multi_armed_bandit.md) / [操作できる図解](../../visualizations/b_21_25/index.html#multi_armed_bandit)
- [焼きなましの近傍管理](neighborhood_manager.md) / [操作できる図解](../../visualizations/b_21_25/index.html#neighborhood_manager)

[全28項目のAPI・条件・比較検証](../b_21_25/README.md)。`python3 verify/run_b_21_25.py` / `--sanitize` / `make verify-b-21-25`。
