# 3.19 集合関数・Bitmask変換

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

S項目を `set_function/` にまとめています。[all.hpp](../../set_function/all.hpp)が全体の入口です。添字をビット集合とみなす変換であり、[3.14の約数／倍数変換](../number_theory_convolution/README.md)とは添字の関係が異なります。

| S項目 | API・usage | 意味・向き | 図解 |
|---|---|---|---|
| Submask列挙 | [for_each_submask](submask_enumeration.md) | T⊆S、数値降順、既定で空集合も含む | [操作](../../visualizations/set_function/index.html#submask) |
| Supermask列挙 | [for_each_supermask](supermask_enumeration.md) | T⊇S、指定した全体集合内で数値昇順 | [操作](../../visualizations/set_function/index.html#supermask) |
| SOS DP | [sos_dp](sos_dp.md) | 全部分集合の和、または可換・結合的merge | [操作](../../visualizations/set_function/index.html#sos) |
| 部分集合ゼータ | [subset_zeta_transform](subset_zeta_transform.md) | F[S]=Σ_(T⊆S)f[T]、ビット0→1 | [操作](../../visualizations/set_function/index.html#subset-zeta) |
| 部分集合メビウス | [subset_mobius_transform](subset_mobius_transform.md) | 上の逆、交互和で復元 | [操作](../../visualizations/set_function/index.html#subset-mobius) |
| 上位集合ゼータ | [superset_zeta_transform](superset_zeta_transform.md) | F[S]=Σ_(T⊇S)f[T]、ビット1→0 | [操作](../../visualizations/set_function/index.html#superset-zeta) |
| 上位集合メビウス | [superset_mobius_transform](superset_mobius_transform.md) | 上の逆、交互和で復元 | [操作](../../visualizations/set_function/index.html#superset-mobius) |

## 共通仕様

- C++17、namespace cp。マスクの列挙はuint64_t、変換の配列はvector<T>でin-placeです。添字0は空集合、長さN=2^nならN−1が全体集合。
- 列挙APIは結果をcallbackへ逐次渡し、内部に蓄積しません。Submaskは0も既定で含み、Supermaskは指定された下位bitsビットの全体集合だけを扱います。0〜64ビットと終端を検査し、1<<64や終端からの折り返しを避けます。callbackの戻り値による打切りはありません。
- 4変換とSOSは**正の2冪長**を要求します。空・非2冪は変更前にinvalid_argument。長さ1（n=0）はそのままです。固定のビット数上限は設けず、メモリとO(n2^n)の処理が収まる範囲で使います。
- ゼータは可換・結合的な加算+=、メビウスは可換な加法群の減算-=が必要です。独自mod型なら合成数法・法1も可で、除算・逆元は使いません。boolは未対応です。
- bool以外の標準整数は中間の加減算を検査し、超過でoverflow_error。符号なし減算も負になる場合は拒否します。途中の例外では以前の更新が残ります。独自型・汎用SOSのmerge内の算術は利用者が管理します。
- SOSの引数1個の版は部分集合ゼータと同じ和。merge指定版はmin/max/OR等も使えます。単位元やT(0)を要求しません。非可換・非結合のmergeに数学的な全部分集合の集約は保証しません。
- 3.12の[dp_optimization/subset_dp.hpp](../../dp_optimization/subset_dp.hpp)にあったfor_each_submask・subset_zeta_transform・subset_mobius_transformの本体を移動しました。旧includeと関数名は保持し、旧ヘッダは新しい3ヘッダを明示includeします。TSPは3.12に残します。整数の超過検査と逆変換のビット降順処理が追加された点は、旧入口にも適用されます。

## 計算量と選び方

- 1つのSubmask列挙はO(2^popcount(mask))、Supermask列挙はO(2^(bits−popcount(mask)))。callbackの費用は別、追加空間O(1)。全maskと全submaskの組は3^nです。
- 変換・SOSはN=2^nとしてO(nN)時間、追加空間O(1)。加減算／mergeは正確にnN/2回です。型の演算費用を掛けます。
- 1つの集合だけ調べる場合は列挙、全ての集合について同じ和・集約を求める場合はSOS／ゼータを使います。集計済みの加法データから元を復元する場合は、同じ向きのメビウスを使います。

## ローカル検証

```sh
python3 verify/run_set_function.py
python3 verify/run_set_function.py --sanitize
python3 verify/run_dp_optimization.py
python3 verify/run_dp_optimization.py --sanitize
node verify/visualizations/set_function_model_test.js
node verify/visualizations/set_function_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で変更できます。9ヘッダ、7使用例、4ジャッジ形式・256ケース、Python任意精度2,428クエリ。2,047基準mask・177,146組、6,654三値表・180ランダム表、全int8加減算、mod12・複素数・単位元不要の型、2^20項の全出力と演算回数を検証します。旧DPカテゴリも通常・サニタイザのdebug/releaseで回帰確認します。オンラインACは未確認です。

図解は7モード。ビット列、列挙の次のmask、各ビット段階の配列、読み取り元と更新先、注目する集合への途中の寄与を表示します。変換4ビット、列挙64ビット・自由ビット8個まで。2,853入力・50,838スナップショットを、処理済みビットを制限した集合の定義から独立に照合します。

## 高度なアルゴリズムの追加

- [Subset Convolution](subset_convolution.md) / [操作できる図解](../../visualizations/advanced/index.html#subset)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。
