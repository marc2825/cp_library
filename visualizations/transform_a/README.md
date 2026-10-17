# 3.16〜3.20 A項目の操作できる図解

[図解を開く](index.html) / [全図解](../index.html) / [API・検証一覧](../../docs/transform_a/README.md) / [一次資料](../../docs/transform_a/references.md)

8モードでJSON入力を変え、最初・前・次・最後・再生を操作できます。SVGの係数表・行列で読み取りと更新先を強調し、各段の説明と履歴表を表示します。外部通信・追加パッケージは不要です。

| モード | 追跡する内部状態 |
|---|---|
| Karatsuba | 係数列の二分、深さ、3再帰積、middleの差分、次数をずらした組立て |
| FWHT | 自然順のbitごとのbutterfly、加減算、逆の2除算 |
| AND/OR/XOR | 変換の向き、各点積、逆変換。operationをand/or/xorへ変更 |
| Ranked Zeta | [rank][mask]の初期配置、bitごとの部分集合和 |
| 逆行列 | [A|I]、pivot、行交換・正規化・消去、特異判定 |
| GF(2) | 01の拡大行列、行XOR、rank、特解、零空間基底 |
| 線形空間の交差 | 独立行基底、[Uᵀ|-Vᵀ]、係数の零空間、元の空間への写像 |
| 特性多項式 | 行列の相似変形、Hessenberg行列、主小行列の多項式 |

## C++との違い

- 教材用の小入力モデルです。係数は-100〜100、配列16項、行列4次、GF(2)は6変数/6方程式、交差の各入力4行/4列まで。JS Numberで厳密に扱える範囲へ制限します。
- Karatsubaは小入力でも再帰を表示するため直接積の閾値を2にし、C++の32と異なります。組立ての式・乗算順は同じです。
- 集合変換は整数modulus=0または0〜101の法。AND/ORとRanked Zetaは合成数法も可、XORの逆は2の逆元が必要です。整数逆FWHTは割り切れを検査します。C++の整数型の超過例外は再現しません。
- 行列は2〜101の素数法だけを扱います。実数epsilon・複素数・独自体型はC++ APIの説明とテストを参照してください。
- GF(2)はbitsetの列を0/1表として見せるモデルで、機械wordへのpackingは表示しません。XOR空間の整数追跡版はC++の専用検証で確認します。
- Ranked Zetaの逆APIはC++使用例で説明し、図解はrank層の順変換を追います。Subset Convolution全体の図解は[既存の図解](../advanced/index.html#subset)へ。

## 検証

```sh
node verify/visualizations/transform_a_model_test.js
node verify/visualizations/transform_a_browser_test.mjs --browser /path/to/chromium
```

モデルは1,924入力・31,226状態・15拒否条件。直接積と符号付きHadamard和、bitwise二重ループ、処理済みbitだけを変えたrank和、各[A|I]状態のL=RA、相似変形の特性多項式、主小行列の置換和、多項式の全係数、有限体の全空間・GF(2)の全解との一致を検証します。状態を変更して他のスナップショットが変わらないことも確認します。

ブラウザは8モードの入力変更・手順移動・再生・エラー復帰・相互リンク・390px表示を検証します。`--screenshot /tmp/transform-a.png`で画像を保存できます。
