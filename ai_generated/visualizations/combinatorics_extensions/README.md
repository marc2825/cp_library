# 組合せ C追加項目の図解

[8モードを開く](index.html) / [C++ API・検証](../../docs/combinatorics_extensions/README.md) / [図解一覧](../README.md)

JSON入力の編集、先頭/前/次/最後、再生、エラー表示に対応します。390px幅ではパネルを縦に配置し、表は横スクロールできます。Young盤と格子路はSVGでセルを表示し、現在の位置を強調します。入力を変えて実行し、サンプルボタンで戻せます。

| モード | 追う状態 | 説明モデルの上限/相違 |
|---|---|---|
| q二項 | q-Pascal各行、q階乗、最初の0 | n≤25、p≤43。C++はq-Lucas |
| 境界付き列 | 許可セル、左/下から来る経路数 | N≤12、高さ≤32。C++は長方形分割とNTT |
| 小素数Stirling | p進分解、周期表、2種の結果 | p≤43、n/kはuint64。大きい整数は文字列指定 |
| ラベルなし木 | 根付き数と無根数 | n≤24、BigIntで厳密 |
| RSK | bumpによるPとQ、逆挿入 | 単語長≤16、整数値のみ。C++は汎用型/biword/行列も対応 |
| Young盤 | 追加可能な角からの全標準盤 | 総セル数≤8 |
| Hook | 各hook長、分母積、標準/半標準数 | 総セル数≤40、法とalphabetはuint64 |
| Lah/Euler | 変換後の係数、逆で復元した指数 | 長さ≤20。Lahは三角表、Eulerは直接漸化式 |

C++と同じ8使用例の出力をモデル・ブラウザの両方で照合します。`node verify/visualizations/combinatorics_extensions_model_test.js` は独立な部分集合列挙、境界付き列の全列挙、Stirling三角表、LNDS、標準盤の順列充填、Eulerのknapsackと4,377比較。ブラウザテストは全8モード、8編集入力、再生/移動、JSONエラー、ローカルリンク、390px幅を確認します。
