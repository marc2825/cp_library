# 3.12 DP・最適化

今回のS項目を `dp_optimization/` にまとめます。C++17・`namespace cp`・外部依存なし、添字は0始まり、区間は半開です。必要なヘッダだけ、または `dp_optimization/all.hpp` をincludeします。

| やりたいこと | API・説明 | 図解 |
|---|---|---|
| 最長増加部分列と復元 | [lis](lis.md) | [LIS](../../visualizations/dp_optimization/index.html#lis) |
| 最長共通部分列と対応添字 | [lcs / lcs_length](lcs.md) | [LCS](../../visualizations/dp_optimization/index.html#lcs) |
| 単位費用の編集距離と操作列 | [edit_distance / edit_alignment](edit_distance.md) | [編集距離](../../visualizations/dp_optimization/index.html#edit) |
| 各品物を高々1個選ぶ | [zero_one_knapsack](zero_one_knapsack.md) | [0-1](../../visualizations/dp_optimization/index.html#zero-one) |
| 各種類を何個でも選ぶ | [unbounded_knapsack](unbounded_knapsack.md) | [完全](../../visualizations/dp_optimization/index.html#unbounded) |
| 個数上限まで選ぶ | [bounded_knapsack](bounded_knapsack.md) | [個数制限](../../visualizations/dp_optimization/index.html#bounded) |
| 0-1部分和の到達可能性 | [BitsetKnapsack](bitset_knapsack.md) | [Bitset](../../visualizations/dp_optimization/index.html#bitset) |
| 区間を二つに分ける最小費用 | [interval_dp](interval_dp.md) | [区間DP](../../visualizations/dp_optimization/index.html#interval) |
| 十進上限以下の整数を状態遷移で数える | [digit_dp](digit_dp.md) | [桁DP](../../visualizations/dp_optimization/index.html#digit) |
| 部分集合列挙・集合変換・TSP | [subset_dp](subset_dp.md) | [部分集合DP](../../visualizations/dp_optimization/index.html#subset) |
| ヒストグラム／二値グリッドの最大長方形 | [largest_rectangle](largest_rectangle.md) | [棒](../../visualizations/dp_optimization/index.html#rectangle) / [グリッド](../../visualizations/dp_optimization/index.html#grid) |

[ナップサックの共通型](knapsack_common.md) / [一次資料・設計判断](references.md) / [動く使用例](../../examples/dp_optimization/) / [全ての図解](../../visualizations/index.html)

## 選び方と条件

ナップサックの価値DPは**重さちょうど**の表を返し、未到達はnulloptです。容量以下の答えは `best_value()`。価値は負でもよく、0-1／個数制限は重さ0も許容します。完全版は重さ正。個数制限は剰余類ごとの単調キューでO(NW)、Bitsetは価値を持たず到達可能性を64ビット単位で更新します。

区間DPは `dp[l][r]=min(dp[l][k]+dp[k][r]+cost(l,k,r))` の二分割テンプレート。桁DPは任意の有限状態とCount型を指定します。部分集合DPは部分集合列挙・ゼータ／メビウス変換と、20頂点までの最小巡回路を提供します。指数時間の処理は入力規模に注意し、各説明の計算量・メモリ量を確認してください。

最適化DPの費用・価値・面積はlong longで、加算・乗算の範囲を検査します。標準整数の桁DPの個数も超過を検出します。コールバック内の計算やジェネリックな集合変換の算術、法を取るCount型は利用者が管理します。全中間値が型に収まる必要があります。前提違反のうちサイズや添字・重さはassert、文字列・行列の形などは例外です。詳細は各APIの条件を参照してください。

LIS・LCS・編集距離・区間分割・TSP・長方形は解の復元にも対応します。同点の選択は特記のない限り辞書順最小を保証しません。旧 `DP/` や `misc/Largest_Rectangle.hpp` のコードは保持し、新カテゴリの利用へ順次移行できます。

## 検証

```sh
python3 verify/run_dp_optimization.py
python3 verify/run_dp_optimization.py --sanitize
node verify/visualizations/dp_optimization_model_test.js
node verify/visualizations/dp_optimization_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`等で変更可能。C++17・警告エラー、14ヘッダ（共通・内部補助含む）の単独include、既存カテゴリ・旧LISとの混在、複数翻訳単位、使用例11本、12ジャッジ形式・576ケースを検証します。オンラインACは未確認です。

- 三値配列9,841個、LCSの二値文字列3,969組、実際の編集グラフ225組。
- 小さな品物列6,175通りと400ランダム入力で全dpセルを個数列挙と比較。
- 200区間結合、桁和の全上限0〜1000・法1〜9、禁止桁と隣接制約、300TSPを全順列と比較。
- 5,058二値グリッドと全ヒストグラムの長方形を全探索。座標・復元経路・操作の正当性も確認。
- 空・0・負値・整数極値／超過・0重さ・巨大個数・64ビット境界・最上位ビット。
- 20万要素／容量、10万桁、18頂点TSP、160要素区間DP、長さ2,000の文字列。

図解は12モード。部分集合DPでは4種類の処理を切替できます。1,784入力・19,679スナップショットを独立オラクルと比較し、走査途中のDP、単調キュー、十進接頭辞、集合状態、64ビット境界も検査します。ブラウザでは操作・再生・入力エラー・リンク・390px表示を確認します。

## 高度なアルゴリズムの追加

- [Li Chao Tree](li_chao_tree.md) / [操作できる図解](../../visualizations/advanced/index.html#lichao)
- [Monotone Minima](monotone_minima.md) / [操作できる図解](../../visualizations/advanced/index.html#monotone)
- [SMAWK](smawk.md) / [操作できる図解](../../visualizations/advanced/index.html#smawk)
- [凸 Min-Plus Convolution](min_plus_convolution.md) / [操作できる図解](../../visualizations/advanced/index.html#minplus)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

## 追加A項目（2026-10-17）

- [Divide and Conquer DP Optimization](divide_conquer_dp.md) — 連続区間への分割DPを段ごとに更新し、選んだ区間を復元します。
- [Knuth Optimization](knuth_optimization.md) — 分割点の二方向の単調性を使い、区間DPを二乗時間にします。
- [Convex Hull Trick DP](convex_hull_trick_dp.md) — 二乗距離のDPを直線の追加・最小値問い合わせへ変形します。
- [Li Chao Tree DP](li_chao_dp.md) — 二乗距離のDPを直線の追加・最小値問い合わせへ変形します。
- [Slope Trick](slope_trick.md) — 整数傾きの凸区分線形関数へhinge・絶対値を加え、最小値DPを更新します。
- [Aliens Trick / WQS Binary Search](aliens_trick.md) — 個数への罰則付き最適化から、ちょうどK個を使う離散凸DPの値を求めます。

[追加分の契約・検証範囲](../math_a/README.md) / [18モードの図解](../../visualizations/math_a/index.html)。追加分は `python3 verify/run_math_a.py` / `--sanitize` で検証します。既存APIとカテゴリ検証は保持します。

## B項目（2026-10-18）

[30項目の対応範囲・比較検証](../b_12_15/README.md) / [一次資料と設計判断](../b_12_15/references.md)。`make verify-b-12-15`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [min_plus_convolution.hpp](../../dp_optimization/min_plus_convolution.hpp) | [Min-Plus Convolution（凸・凹）](min_plus_convolution_b.md) | [図解](../../visualizations/b_12_15/index.html#min_plus_convolution) |
| [monge_shortest_path.hpp](../../dp_optimization/monge_shortest_path.hpp) | [Monge Shortest Path](monge_shortest_path.md) | [図解](../../visualizations/b_12_15/index.html#monge_shortest_path) |
| [monge_shortest_path_k_edges.hpp](../../dp_optimization/monge_shortest_path_k_edges.hpp) | [Mongeの辺数制約付き最短路](monge_shortest_path_k_edges.md) | [図解](../../visualizations/b_12_15/index.html#monge_shortest_path_k_edges) |
| [larsch.hpp](../../dp_optimization/larsch.hpp) | [LARSCH](larsch.md) | [図解](../../visualizations/b_12_15/index.html#larsch) |
| [online_offline_dp.hpp](../../dp_optimization/online_offline_dp.hpp) | [オンライン・オフライン変換（CDQ）](online_offline_dp.md) | [図解](../../visualizations/b_12_15/index.html#online_offline_dp) |
| [branch_bound_knapsack.hpp](../../dp_optimization/branch_bound_knapsack.hpp) | [分枝限定ナップサック](branch_bound_knapsack.md) | [図解](../../visualizations/b_12_15/index.html#branch_bound_knapsack) |
| [four_russians.hpp](../../dp_optimization/four_russians.hpp) | [Four Russians法（Boolean行列積）](four_russians.md) | [図解](../../visualizations/b_12_15/index.html#four_russians) |
