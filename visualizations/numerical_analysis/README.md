# 数値解析の操作できる図解

[開く](index.html) / [API・検証](../../docs/numerical_analysis/README.md) / [全図解](../index.html)

Y全5項目を11モードで扱います。JSONを変更して実行し、「次へ」「戻る」「最後」、自動再生で手順を追えます。曲線と評価点、積分区間の境界、補償和の途中値、Sturmの区間内根数、行列の非零成分を表示します。図の横軸・縦軸は別の尺度で拡大します。

| 対応 | 図解の入力・状態 |
|---|---|
| Y-01 | 二次式の係数、二分・Newton・Brentの多項式と区間・初期値・許容誤差・反復予算 |
| Y-02 | Simpsonの多項式と偶数分割数、適応版のexpまたは多項式と深さ・許容誤差 |
| Y-03 | Kahan/Neumaierの途中和、最大値を引いたexpの寄与、絶対＋相対比較 |
| Y-04 | 整数係数多項式のSturm列・二分・根区間とcount |
| Y-05 | 三重対角係数と右辺、隣接行交換・第2上対角・後退代入 |

C++との違い：通常の計算はJavaScript Numberです。C++のlong doubleの精度・指数範囲、評価予算、全ての非有限値の規約は再現しません。有限数をJSONで入力します。二次判別式とSturmの符号は既存の[BigInt有理数](../computational_geometry/model.js)を共有します。Sturmは整数係数・次数8以下・係数絶対値100万以下で、許容誤差1e−12〜1・分割1000回以下です。分割状態の表示は先頭100回まで、計算は指定予算まで継続します。根の個数は厳密に数えます。

求根図解のtoleranceは区間の絶対・相対許容誤差と残差許容誤差を兼ねます。Newtonは残差のみで成功判定します。反復は160回までです。積分図解のtoleranceは絶対・相対の両方で、適応版は深さ16、分割256回まで。誤差の大きい区間から分割する順序はC++と共通で、図解は区間配列を走査します。Simpsonは2〜64の偶数分割です。通常の係数・区間は絶対値100万以下、expの上端は20以下です。

和とLogSumExpは64項まで。補償和の棒の高さは符号付きlog10(1+|x|)です。LogSumExpの図解は最大項以外をNumberで直接足します。行列は12行までで、見える密行列を保持しつつ帯幅2の更新を行い、pivot閾値は0です。C++のO(n)保存や因子の再利用はAPI説明を参照してください。数値型の表現範囲を超える結果は入力エラーとして表示します。

[11サンプルの参照値](../../verify/numerical_analysis/visualization_expected.json)は実際のC++例から取得し、モデルとブラウザで照合します。モデルでは解析的な根・積分・生成した行列の解と独立比較し、ブラウザでは全11モードの変更入力・移動・再生・エラー・リンク・390px表示を確認します。

```sh
node verify/visualizations/numerical_analysis_model_test.js
node verify/visualizations/numerical_analysis_browser_test.mjs --browser /path/to/chromium --screenshot /tmp/numerical.png
```
