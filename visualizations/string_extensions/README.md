# 文字列・圧縮索引 Sの図解（11モード）

[開く](index.html) / [API・検証](../../docs/string_extensions/README.md) / [全図解](../README.md)

JSONを編集し、文字の遷移・候補区間・回文・周期を前後移動・再生で確認できます。文字列はUTF-8 byte列、数値配列は0〜255を直接指定します。文字数ではなくbyte位置を扱い、図にはhexも表示します。外部通信・CDNは不要です。

一般の文字列は48byte以下、BWT/FMは24byte以下、LCSは各16byte以下、SAMは各24byte・文書合計48byte以下です。両端操作は80回以下で、追加値は1byte（整数も可）。非ASCIIの部分文字列の結果はhexで表示します。

BWTの整列、FMの区間縮小、seaweedの交換、SAMの状態生成を追います。図解のrankは直接計数、回文は全区間の直接列挙、Zは直接比較、SAMのk番目は部分文字列集合を使う小規模な説明モデルです。C++のpacked rank、surface更新、ancestor探索、経路数の飛越しとは実行手順が異なります。LCSのサンプルでは複数のqueryに同じ境界値を使うため、A/B双方に入る境界を指定します。

11使用例の結果をC++実行と共有し、モデル検証とChromiumの入力編集・ナビゲーション・再生・エラー復帰・390px表示で確認します。

```sh
node verify/visualizations/string_extensions_model_test.js
node verify/visualizations/string_extensions_browser_test.mjs --browser /path/to/chromium
```
