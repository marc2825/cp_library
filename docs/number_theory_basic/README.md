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
