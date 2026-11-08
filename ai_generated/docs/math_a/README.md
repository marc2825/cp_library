# 3.12〜3.15 A項目の補完

3.12〜3.15の不足19掲載箇所を補完する追加です。CRTと一般化CRTは同じ公開APIを使います。16の新しい公開ヘッダと18の個別説明・使用例を用意し、既存CHT/Li ChaoはDPへの組合せと比較検証を追加しました。3.14の全5項目、既存Monotone Minima・SMAWK・SOS・補間・二項係数は共有します。

[操作できる18モード](../../visualizations/math_a/index.html) / [一次資料](references.md)

## 追加したAPIと使用例

| 項目 | 内容 |
|---|---|
| [Divide and Conquer DP Optimization](../dp_optimization/divide_conquer_dp.md) | 連続区間への分割DPを段ごとに更新し、選んだ区間を復元します。 |
| [Knuth Optimization](../dp_optimization/knuth_optimization.md) | 分割点の二方向の単調性を使い、区間DPを二乗時間にします。 |
| [Convex Hull Trick DP](../dp_optimization/convex_hull_trick_dp.md) | 二乗距離のDPを直線の追加・最小値問い合わせへ変形します。 |
| [Li Chao Tree DP](../dp_optimization/li_chao_dp.md) | 二乗距離のDPを直線の追加・最小値問い合わせへ変形します。 |
| [Slope Trick](../dp_optimization/slope_trick.md) | 整数傾きの凸区分線形関数へhinge・絶対値を加え、最小値DPを更新します。 |
| [Aliens Trick / WQS Binary Search](../dp_optimization/aliens_trick.md) | 個数への罰則付き最適化から、ちょうどK個を使う離散凸DPの値を求めます。 |
| [Euler φ](../number_theory_basic/euler_phi.md) | 単一の64bit整数と、上限までの全整数についてEulerのφ関数を求めます。 |
| [Möbius μ](../number_theory_basic/mobius.md) | 平方因子の有無と素因数個数から、Möbius関数を計算します。 |
| [CRT / 一般化中国剰余定理](../number_theory_basic/crt.md) | 互いに素でない正の法も含む連立合同式を解きます。 |
| [Garner](../number_theory_basic/garner.md) | 任意個の互いに素な法のCRT解を、指定した法で取得します。 |
| [Floor Sum](../number_theory_basic/floor_sum.md) | 負の係数にも対応し、一次式を割って床を取った総和を求めます。 |
| [Primitive Root](../number_theory_basic/primitive_root.md) | 素数を法とした乗法群の最小の正の原始根を求めます。 |
| [Stirling数（第1種・第2種）](../combinatorics/stirling.md) | 順列のcycle数と集合分割のブロック数を数える三角表を構築します。 |
| [Bell数](../combinatorics/bell.md) | N要素の集合を、順序のない非空ブロックへ分ける方法を数えます。 |
| [整数の分割数](../combinatorics/partition_numbers.md) | 正整数の和による整数分割を、和の順序を区別せずに数えます。 |
| [Montmort数 / Derangements](../combinatorics/derangements.md) | 固定点が一つもない順列の個数を求めます。 |
| [Burnsideの補題](../combinatorics/burnside.md) | 群作用で同一視される対象の数を、固定点数の平均から求めます。 |
| [Pólya Enumeration / Cycle Index](../combinatorics/polya.md) | 群の全置換からcycle indexを構築し、色数や重みを代入して軌道を数えます。 |

## 契約と設計判断

- D&Cは最左argminの単調性、Knuthは二方向の単調性が必要です。DP段と半開区間の復元を公開し、到達不能値に有限のINFを使いません。
- WQSは整数値の離散凸なF(k)と、同値なら個数が多いoracleが前提です。任意のDPでexact-Kが得られる契約ではありません。探索端点の個数がKを挟まなければnulloptです。
- Slope Trickは単位hingeの二heap版。prefix/suffix最小化・区間shift・非破壊merge・点評価を含みます。重みをまとめた高度版は対象外です。
- φ/μの篩は既存線形SPF、単一64bit値と原始根の因数分解は既存Miller–Rabin/Pollard–Rhoを共有。原始根は素数法です。
- CRTは非互いに素・負剰余・法1・空入力に対応し、矛盾はnullopt、prefixのlcm超過は例外。Garnerは任意長のpairwise coprimeな法と、入力法と共通因子があってもよい出力法です。畳み込みの固定3法内部処理は保持します。
- floor_sumはn,mが32bit範囲、a,bはsigned 64bit。負数の床と中間値超過を検査し、最終値が相殺される場合も正規化の途中で超過すれば例外です。__int128には依存しません。
- Stirlingは第1種の符号あり/なしと第2種、Bell、整数分割、Montmortは合成数法・法1にも対応。Burnside/Pólyaのmod平均には群の位数の逆元が必要です。Burnsideのexact版はuint64総和を整数で割ります。CycleIndexは置換の妥当性を検査し、群の全要素の列挙は呼出側の責任です。
- 旧CHTのクラス名・グローバルAPIを維持し、標準名を明示修飾、特殊化関数をinlineへ変更しました。複数翻訳単位の重複定義とstd名の混入を防ぎます。旧CHTの未検査の算術条件は個別説明に明記します。

## 検証

```sh
python3 verify/run_math_a.py
python3 verify/run_math_a.py --sanitize
make verify-math-a
node verify/visualizations/math_a_model_test.js
node verify/visualizations/math_a_browser_test.mjs --browser /path/to/chromium
```

18公開入口の単独・重複include（debug/release）、全カテゴリ混在の逆順includeと複数翻訳単位、18の使用例と説明中の完全一致、8ジャッジ形式プログラム各24ケースを検証します。C++の独立した差分検証は分割/区間DPの素朴解・復元・CHTとLi Chaoの二乗遷移・離散凸WQS・凸関数の全整数格子、互いに素な個数・約数のμ和・CRT/Garner全探索・床の直接和・原始根の位数、順列cycle・集合/整数分割・完全順列・彩色軌道の全列挙を使います。Python任意精度でもfloor_sum/CRT/Garnerの5,000ケースを比較します。

既存の `run_dp_optimization.py`、`run_number_theory_basic.py`、`run_number_theory_convolution.py`、`run_combinatorics.py`、`run_advanced.py` を回帰検証します。オンラインACは未確認です。サニタイザとChromiumには環境のptrace/socket制限の外での実行が必要になる場合があります。

図解は小入力の説明用モデルで、CHTは直線の直接評価、φ/μはSPFの教材用構築、Pólyaは輪の回転/反転を例にします。C++と図解それぞれを独立した素朴解で検証し、[モデルとの差](../../visualizations/math_a/README.md)を明記しています。
