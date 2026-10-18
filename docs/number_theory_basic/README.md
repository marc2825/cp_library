# 3.13 数論：基本整数演算

C++17・`namespace cp`。まとめて使う場合は [all.hpp](../../number_theory_basic/all.hpp)、個別のヘッダも単独でincludeできます。

| 用途 | 実装と説明 | 主な条件・戻り値 |
|---|---|---|
| 最大公約数・最小公倍数 | [GCD/LCM](gcd_lcm.md) | 符号付き入力、符号なしの非負結果。LCMの超過はnullopt |
| Bézout係数 | [拡張Euclid](extended_gcd.md) | a*x+b*y=g、負数・最小整数も可 |
| 法を取った累乗・乗算 | [mod累乗](mod_pow.md) | 指数>=0、法>=1、積のあふれを回避 |
| 法を取った除算 | [mod逆元](mod_inverse.md) | 逆元なしはnullopt、合成数法も可 |
| 範囲内の素数判定・列挙 | [素数篩](prime_sieve.md) | 上限を含む表、0・1は非素数 |
| 多数の素因数分解 | [SPF](smallest_prime_factor.md) | 線形前処理、表の上限以下を分解 |
| 単発の素因数分解 | [試し割り](prime_factorization.md) | 正整数、(素数,指数)を昇順 |
| 正の約数 | [約数列挙](divisors.md) | 正整数、重複なし・昇順 |

[操作できる図解：8モード](../../visualizations/number_theory_basic/index.html) / [使用例](../../examples/number_theory_basic/) / [一次資料・設計判断](references.md)

篩とSPFの上限は両端を含む `[0,N]` です。分解・約数列挙の0以下は例外で拒否します。1の素因数分解は空、約数は `{1}` です。GCDと拡張Euclidは `LLONG_MIN` の大きさも返すためGCDを符号なし型にします。一般の符号なし64ビット入力を受け取るAPIではありません。

法は `long long` の正整数全域を扱います。一般の大きな法の乗算には最大約63回の加算が必要ですが、法が2^32以下なら積を直接計算できます。試し割り・約数列挙はO(√n)なので、大きい素数が入力される用途では計算量を先に確認してください。

旧 `number_theory/` の関数・ファイルは保持しています。新APIは名前空間、上限・例外条件、符号・戻り値を明示しており、旧関数の別名ではありません。旧Eratosthenesの0・1入力や、旧分解の0入力の動作を引き継ぐものではありません。

## 検証

```sh
python3 verify/run_number_theory_basic.py
python3 verify/run_number_theory_basic.py --sanitize
node verify/visualizations/number_theory_basic_model_test.js
node verify/visualizations/number_theory_basic_browser_test.mjs --browser /path/to/chromium
```

`make verify-number-theory-basic`、全カテゴリの図解モデルは `make verify-visualizations`。コンパイラは `--compiler clang++` 等で変更できます。

9ヘッダの単独・逆順・混在includeと複数翻訳単位、8使用例、9ジャッジ形式・432ケース、debug/release両方の小入力全探索とPython任意精度20,630ケース。整数極値・LCM超過・巨大指数・合成数の法・平方数・200万要素の篩を含みます。旧実装は正の小入力で比較します。オンラインACは未確認です。

## 追加A項目（2026-10-17）

- [Euler φ](euler_phi.md) — 単一の64bit整数と、上限までの全整数についてEulerのφ関数を求めます。
- [Möbius μ](mobius.md) — 平方因子の有無と素因数個数から、Möbius関数を計算します。
- [CRT / 一般化中国剰余定理](crt.md) — 互いに素でない正の法も含む連立合同式を解きます。
- [Garner](garner.md) — 任意個の互いに素な法のCRT解を、指定した法で取得します。
- [Floor Sum](floor_sum.md) — 負の係数にも対応し、一次式を割って床を取った総和を求めます。
- [Primitive Root](primitive_root.md) — 素数を法とした乗法群の最小の正の原始根を求めます。

[追加分の契約・検証範囲](../math_a/README.md) / [18モードの図解](../../visualizations/math_a/index.html)。追加分は `python3 verify/run_math_a.py` / `--sanitize` で検証します。既存APIとカテゴリ検証は保持します。

## B項目（2026-10-18）

[30項目の対応範囲・比較検証](../b_12_15/README.md) / [一次資料と設計判断](../b_12_15/references.md)。`make verify-b-12-15`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [discrete_log.hpp](../../number_theory_basic/discrete_log.hpp) | [Discrete Log（BSGS）](discrete_log.md) | [図解](../../visualizations/b_12_15/index.html#discrete_log) |
| [mod_sqrt.hpp](../../number_theory_basic/mod_sqrt.hpp) | [mod平方根（Tonelli–Shanks）](mod_sqrt.md) | [図解](../../visualizations/b_12_15/index.html#mod_sqrt) |
| [integer_kth_root.hpp](../../number_theory_basic/integer_kth_root.hpp) | [整数k乗根（数論入口）](integer_kth_root.md) | [図解](../../visualizations/b_12_15/index.html#integer_kth_root) |
| [mod_tetration.hpp](../../number_theory_basic/mod_tetration.hpp) | [modテトレーション](mod_tetration.md) | [図解](../../visualizations/b_12_15/index.html#mod_tetration) |
| [quotient_enumeration.hpp](../../number_theory_basic/quotient_enumeration.hpp) | [商列挙（数論入口）](quotient_enumeration.md) | [図解](../../visualizations/b_12_15/index.html#quotient_enumeration) |
| [linear_congruence.hpp](../../number_theory_basic/linear_congruence.hpp) | [線形合同方程式](linear_congruence.md) | [図解](../../visualizations/b_12_15/index.html#linear_congruence) |
| [lucas.hpp](../../number_theory_basic/lucas.hpp) | [Lucasの定理](lucas.md) | [図解](../../visualizations/b_12_15/index.html#lucas) |
| [prime_count.hpp](../../number_theory_basic/prime_count.hpp) | [素数カウント（共通入口）](prime_count.md) | [図解](../../visualizations/b_12_15/index.html#prime_count) |
