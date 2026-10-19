# 多倍長整数の図解

[図を開く](index.html) / [C++ API・使用例・検証](../../docs/basic_utility/big_integer.md) / [図解一覧](../index.html)

JSON入力を編集し、先頭・戻る・次へ・最後・再生を操作できます。数値は文字列で指定し、図解では最大36桁に制限します。C++側にはこの固定桁数制限はありません。

| モード | 示す内容 | C++実装との関係 |
|---|---|---|
| parse | 符号、10進／16進のchunk、累積値 | C++と同じ最大9桁／8桁の区切りで読み取る |
| arithmetic | `op`を`+`・`-`・`*`にして、繰り上がり・繰り下がり・部分積を見る | 基数1000の筆算モデル。BoostのlimbやKaratsubaの再現ではない |
| division | 桁を下ろす筆算、商・余りと符号の復元 | `a=q*b+r`、0方向への商の切り捨てはC++と共通 |
| gcd | 絶対値から余りを繰り返し、GCDへ到達する | Euclid法の説明モデル。Boost内部のGCD手順とは異なる |

整数計算にJavaScript BigIntを使い、演算モードの筆算は1000未満の各桁をNumberで扱います。内部の1桁計算は安全な整数範囲に収まります。Boostの性能、メモリアロケータ、例外発生位置を表す図ではありません。16進出力は符号付きの絶対値表記であり、2の補数表現ではありません。

## 検証

```sh
node verify/visualizations/big_integer_model_test.js
node verify/visualizations/big_integer_browser_test.mjs --browser /path/to/chromium
python3 verify/run_big_integer.py
```

モデルはネイティブBigIntとの4,200比較、90符号境界ケース、スナップショット独立性、4つのC++参照値、入力エラーを検証します。ブラウザでは4モードの操作、5種類の編集入力、0除算エラー、ローカルリンク、390px表示を確認します。`--screenshot /tmp/big-integer.png`でdesktop/mobile画像も保存できます。
