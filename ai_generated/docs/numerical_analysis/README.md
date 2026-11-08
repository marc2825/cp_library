# 数値解析 Y-01〜Y-05

[全ヘッダ](../../numerical_analysis/all.hpp) / [図解11モード](../../visualizations/numerical_analysis/index.html) / [一次資料](references.md)

実数の求根、積分、安定な集計、実根分離、三重対角ソルバを `cp::numerical` にまとめました。C++17・long doubleを使います。既存のBigRationalと幾何の二進有理数変換を共有し、一般の密行列Gaussや有限体の多項式とは別の入口です。

| ID | 説明・使用例 | 主な処理 |
|---|---|---|
| Y-01 | [安定な二次方程式](quadratic.md) | 符号の確定した判別式とq=−(b+sign(b)√D)/2から、q/aとc/qを求めて桁落ちを避ける。 |
| Y-01 | [二分法による求根](bisection.md) | 符号の異なる端点を維持し区間を半減する。 |
| Y-01 | [Newton法](newton.md) | 接線のx切片へ移り、関数値・微分・進行の停止を調べる。 |
| Y-01 | [Brent法](brent.md) | 逆二次補間・割線の候補を進行条件で採否し、不適切な候補では二分法へ戻る。 |
| Y-02 | [複合Simpson積分](simpson.md) | 隣接3点の放物線を積分し、1・4・2・…・4・1の重みを補償和で集める。 |
| Y-02 | [適応Simpson積分](adaptive_simpson.md) | 粗いSimpsonと二分したSimpsonの差/15を推定誤差にし、全体の誤差予算を超える間、最大誤差の区間を細分化する。 |
| Y-03 | [Kahan和・Neumaier補償和](summation.md) | 加算で失われた下位桁を別に保持し、次の加算または最後の結果へ補う。 |
| Y-03 | [Log-Sum-Exp](log_sum_exp.md) | 最大値を引いてexpのoverflowを防ぎ、最大項以外の和にlog1pを使う。 |
| Y-03 | [絶対＋相対誤差比較](tolerance.md) | 値を共通の尺度で正規化し、差や相対誤差の積のoverflowを避けて比較する。 |
| Y-04 | [多項式の実根列挙・Sturm分離](polynomial_roots.md) | fとf′のGCDで重複因子を除き、Sturm列の符号変化数の差を保ちながら二分する。 |
| Y-05 | [pivot付き三重対角ソルバ](tridiagonal.md) | 最大絶対値のpivotを隣接2行から選び、交換で生じる第2上対角も保存して後退代入する。 |

## 共通の数値契約

有限値の入力・許容誤差・次元を満たさない呼び出しはinvalid_argument、次数上限はlength_error。計算を始めた後の予算超過・非有限値・特異pivotは結果のstatusで返します。補償和とLogSumExpの内部overflowだけはoverflow_errorです。ユーザーcallbackが投げた例外は伝播します。`Status` の列挙値を `status_name` で文字列化できます。

`converged` の意味はAPIごとに異なります。求根はcriterion、積分はerror_estimate、実根分離は有理区間、行列は残差も確認してください。誤差推定・近似解を厳密な前進誤差保証とは扱いません。浮動小数点の桁数・指数範囲は環境依存です。通常の丸めモードを前提とし、`-ffast-math` や浮動小数点例外トラップは使いません。予算は評価回数・分割回数を数え、callbackの実行時間や任意精度整数のbit成長を制限しません。

[二進値とBigRationalの共通変換](../basic_utility/binary_rational.md)を使う二次方程式・多項式実根・all.hppにはBoostヘッダが必要です。長さ0、反転積分区間、全0多項式などの扱いは個別説明に記載しています。

## 検証

```sh
make verify-numerical-analysis
python3 -m tools.cp_tools.suite --runners numerical_analysis
python3 -m tools.cp_tools.suite --stdlib-debug --runners numerical_analysis
python3 -m tools.cp_tools.suite --sanitize --runners numerical_analysis
node verify/visualizations/numerical_analysis_model_test.js
node verify/visualizations/numerical_analysis_browser_test.mjs --browser /path/to/chromium
```

[manifest](../../verify/numerical_analysis/manifest.json)に11使用例・公開ヘッダ・説明・C++出力を対応付けています。runnerは単独／重複include、全カテゴリ2翻訳単位、Debug/NDEBUG、500件の独立Python Decimal/Fraction比較、境界テストを行います。厳密Gaussとの160系比較、20万行の帯行列、300求根比較、近接根・重根・予算終了、非有限値・subnormal・相殺を含みます。二進有理変換の共通化は既存Hテストでも回帰検証します。オンラインACは未確認です。

図解はJavaScript Numberと小規模入力の説明モデルです。C++のlong doubleのbit数とは異なります。詳細は[図解の対応範囲](../../visualizations/numerical_analysis/README.md)。2026-10-28に図解モデル720比較、ブラウザ全11モード・11変更入力・再生／移動／エラー・390px表示が成功し、Brent・適応積分・三重対角の画像を確認しました。
