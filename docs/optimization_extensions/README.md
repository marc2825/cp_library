# 最適化O全8項目・ゲームB-20

[統合一覧](../remaining_work.md#o-01) / [組合せ最適化](../combinatorial_optimization/README.md) / [ゲーム理論](../game_theory/README.md) / [9モードの図解](../../visualizations/optimization_extensions/index.html) / [一次資料](references.md)

O-01〜O-04・O-20〜O-23の8項目とB-20を整備しました。O-05〜O-19、B-01〜B-19の行はありません。C++17、namespace cp。Oはcombinatorial_optimization/all.hpp、Bはgame_theory/all.hppへ追加しています。

| ID | API・説明 | 主な用途 |
|---|---|---|
| O-01 | [二段階Simplex・線形計画](linear_program.md) | 資源配分、混合戦略のLP、連続緩和と双対証明。 |
| O-02 | [固定1〜3次元の厳密LP](low_dimensional_lp.md) | 2D/3Dの半空間制約、退化した領域上の線形目的。 |
| O-03 | [Matroid Union・Partition](matroid_union.md) | 辺の複数森林への分配、異なる独立条件を持つ担当先への割当。 |
| O-04 | [分数計画・Dinkelbach・比率二分](fractional_programming.md) | 利益/費用や総得点/総量の比率を最適化する問題。 |
| O-20 | [重み付きマトロイド交差](weighted_matroid_intersection.md) | 色制約付き森林、容量付き二部割当、重みと要素数の両方を調べる問題。 |
| O-21 | [Base Polyhedron・Lovász拡張](base_polyhedron.md) | 劣モジュラ関数の基の生成、Lovász拡張、凸最適化の線形oracle。 |
| O-22 | [劣モジュラ最適化](submodular_optimization.md) | カットにモジュラ項を足した集合関数の最小化、被覆の個数制限付き近似最大化。 |
| O-23 | [有限上下限付き整数線形計画](integer_program.md) | 小規模な整数資源配分、有限範囲の線形制約、LP緩和で枝を削れる探索。 |
| B-20 | [Q(ω)の超現実数・無限小](omega_rational.md) | 無限大・無限小を含む数の比較、数としてのゲーム値の和・差、記号的な漸近順序。 |

## 共通仕様・共有部分

入力配列は保持し、返す解は独立した値です。次元・負のサイズ・非有限入力などはinvalid_argument、明示上限はlength_error、0除算はdomain_error。oracleからの例外は伝播します。マトロイド公理や劣モジュラ性は呼び出し側の条件で、全公理の検査はしません。oracleへ渡す要素は0始まり・重複なしで、順序に依存してはいけません。

BigRationalを使う厳密LP・低次元LP・基多面体・劣モジュラ最小化・整数計画・Q(ω)にはBoostヘッダが必要です。Simplexは浮動型も選べますが、epsilonによる分類を厳密証明とは扱いません。重み付き交差は既存の検査付き128bit、分数計画はYのlong double許容誤差を使います。

既存のGraphicMatroid・PartitionMatroid・Gauss–Jordan・BigRational・PartizanOutcomeを共有します。有理多項式の共通処理は[basic_utility/detail/rational_polynomial.hpp](../../basic_utility/detail/rational_polynomial.hpp)へ抽出し、Yの[Sturm実根分離](../numerical_analysis/polynomial_roots.md)とQ(ω)が同じ除算・GCDを使います。

低次元LPの基底列挙は固定次元dで制約数mのO(m^(d+1))規模です。Simplex、cutting-planeによる一般劣モジュラ最小化、整数計画には指数時間の最悪ケースがあります。予算終了・候補解・証明済みの最適解を区別するAPIです。回数予算は任意精度演算のbit成長やcallbackの実行時間を制限しません。

## 検証

```sh
make verify-optimization-extensions
python3 -m tools.cp_tools.suite --runners optimization_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners optimization_extensions
python3 -m tools.cp_tools.suite --sanitize --runners optimization_extensions
node verify/visualizations/optimization_extensions_model_test.js
node verify/visualizations/optimization_extensions_browser_test.mjs --browser /path/to/chromium
```

[manifest](../../verify/optimization_extensions/manifest.json)に9項目のヘッダ・説明・実行例・参照出力を対応付けています。単独／重複include、全カテゴリ2翻訳単位、Debug/NDEBUGで検証します。C++は240件のLP頂点列挙・主双対/Farkas/ray条件・浮動比較、80低次元LP、400重み付き交差・100unionの全探索、100劣モジュラ最小値、120整数計画、200比率、200組のQ(ω)体恒等式を比較します。Pythonは独立Fraction・全列挙で540件を比較します。

図解は小規模の説明モデルです。入力・予算を変更してpivot・交換路・上下界・無限小を追えます。C++との精度・入力範囲の違いは[図解README](../../visualizations/optimization_extensions/README.md)を参照してください。オンラインACとGitHub Actions上の実行は未確認です。

## 最終検証（2026-10-29確認）

実行時刻と対象ソースは各記録に保存しています。全5記録が最終ソースfingerprint `eba1d4a43287404df4e2b7bfe2fb4a0a15efec1c247fe7880cbd62d47f4ea969`（1988ファイル）に一致しています。

| 対象・構成 | 結果 | 実行時間 | 記録 |
|---|---|---:|---|
| optimization_extensions / standard | 成功 | 60.2秒 | [JSON](../../verification/evidence/optimization_extensions/standard.json) |
| optimization_extensions / stdlib-debug | 成功 | 75.9秒 | [JSON](../../verification/evidence/optimization_extensions/stdlib-debug.json) |
| optimization_extensions / sanitize | 成功 | 111.1秒 | [JSON](../../verification/evidence/optimization_extensions/sanitize.json) |
| numerical_analysis / standard | 成功 | 46.9秒 | [JSON](../../verification/evidence/numerical_analysis/standard.json) |
| b_21_25 / standard | 成功 | 100.8秒 | [JSON](../../verification/evidence/b_21_25/standard.json) |

O/Bの各構成でDebug/NDEBUGの単体テストと独立Python比較540件ずつが成功しました。9使用例と文書内C++の一致、10ヘッダの単独/重複include、全カテゴリを含む2翻訳単位も確認済みです。LPの非有限計算、1D/退化3D、Lovász拡張の指示ベクトル、基多面体の最小化、劣モジュラ近似最大化、空整数計画、各予算終了、Q(ω)のmove元・自己代入も検査しました。

図解モデルは1,049入力・8,562状態・3,610独立比較と12拒否条件が成功。Chromiumは9モード・9変更入力、前後移動・再生・エラー復帰・ローカルリンク・390px幅が成功しました。Simplexのpivot、重み付き交差の交換路、整数計画の分枝領域をデスクトップ/モバイル画像でも確認しました。ChromiumとLeakSanitizerは実行環境のsocket/ptrace制限に対応し、承認レビュー付きで実行しています。

共有の有理多項式処理を変更したYと、既存のマトロイド/ゲームを含むb_21_25を通常構成で回帰しました。進捗245行の件数、文書リンク、workflow構文、出典台帳、差分も確認しています。オンラインAC・GitHub Actions上の実行は未確認です。
