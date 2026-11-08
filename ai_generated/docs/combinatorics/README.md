# 3.15 組合せ論・数え上げ

C++17・`namespace cp`。[all.hpp](../../combinatorics/all.hpp)でまとめて、個別のヘッダで必要な機能だけを使えます。

| 求めたい数 | 実装・説明 | 主な条件 |
|---|---|---|
| 階乗・逆階乗 | [FactorialTable / factorials](factorials.md) | 逆階乗には上限の階乗が法と互いに素であることが必要 |
| 二項係数 | [binomial / PascalBinomial](binomial.md) | 高速版と、合成数法にも対応するO(N²)表 |
| 順列・組合せ | [permutations / combinations](permutation_combination.md) | nPkは順序付き、nCkは順序を無視 |
| Catalan数 | [catalan](catalan.md) | 2nまで前計算。二項係数の差で計算 |
| 重複組合せ | [multiset_combination](multiset_combination.md) | n種類からk個。H(0,0)=1、H(0,k>0)=0 |
| 包除原理 | [union / complement](inclusion_exclusion.md) | 選んだ集合の共通部分を返すコールバック |

[操作できる図解：7モード](../../visualizations/combinatorics/index.html) / [動く使用例](../../examples/combinatorics/) / [一次資料・設計判断](references.md)

通常は `FactorialTable(max_n,modulus)` を1回作り、`binomial(table,n,k)` 等を繰り返し問い合わせます。既定の法は998244353。素数法でも上限が法以上になると階乗の逆元が存在しなくなります。構築時に検出し、`domain_error`で拒否します。小さい上限で任意の正の法を使う場合は `PascalBinomial`。階乗だけなら `factorials` 関数は逆元を要求しません。

法1は結果を全て0にする規約です。高速版の積は3.13の安全なmod乗算・逆元を再利用し、64ビットの正の法まで対応します。一般の大きい法の問い合わせはO(log m)、m<=2^32ならO(1)です。通常の整数の割り算でmod下の除算を置き換えないでください。

包除原理は和集合と、その補集合を分けています。補集合のmask=0は全体集合です。標準整数では途中の範囲超過を検出します。途中で負になる場合があるため、符号付き整数かmod型を使うのが基本です。部分集合の列挙は指数時間で、63集合という値はビット表現上の上限です。

旧組合せ計算はリポジトリ直下の `depricated/Combination/` に保存しています。公開フィールドや旧関数の別名へは置換していません。巨大nのLucas・任意法の高速二項係数や、具体的な順列の列挙は今回のAPIに含めません。

## 検証

```sh
python3 verify/run_combinatorics.py
python3 verify/run_combinatorics.py --sanitize
node verify/visualizations/combinatorics_model_test.js
node verify/visualizations/combinatorics_browser_test.mjs --browser /path/to/chromium
```

`make verify-combinatorics`、全カテゴリの図解モデルは `make verify-visualizations`。追加のPython/npmパッケージは不要です。

8ヘッダの単独・逆順・混在includeと複数翻訳単位、6使用例、6ジャッジ形式・384ケース。361個の有効な階乗表と889個の拒否ケース、69,905集合族、Python任意精度186ブロック・6,531出力、小さい全列挙と独立した直接計算で比較します。200万階乗・1500行のPascal表・22集合の包除、0・整数極値・合成数法を含みます。オンラインACは未確認です。

## 追加A項目（2026-10-17）

- [Stirling数（第1種・第2種）](stirling.md) — 順列のcycle数と集合分割のブロック数を数える三角表を構築します。
- [Bell数](bell.md) — N要素の集合を、順序のない非空ブロックへ分ける方法を数えます。
- [整数の分割数](partition_numbers.md) — 正整数の和による整数分割を、和の順序を区別せずに数えます。
- [Montmort数 / Derangements](derangements.md) — 固定点が一つもない順列の個数を求めます。
- [Burnsideの補題](burnside.md) — 群作用で同一視される対象の数を、固定点数の平均から求めます。
- [Pólya Enumeration / Cycle Index](polya.md) — 群の全置換からcycle indexを構築し、色数や重みを代入して軌道を数えます。

[追加分の契約・検証範囲](../math_a/README.md) / [18モードの図解](../../visualizations/math_a/index.html)。追加分は `python3 verify/run_math_a.py` / `--sanitize` で検証します。既存APIとカテゴリ検証は保持します。

## B項目（2026-10-18）

[30項目の対応範囲・比較検証](../b_12_15/README.md) / [一次資料と設計判断](../b_12_15/references.md)。`make verify-b-12-15`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [eulerian_numbers.hpp](../../combinatorics/eulerian_numbers.hpp) | [Eulerian数](eulerian_numbers.md) | [図解](../../visualizations/b_12_15/index.html#eulerian_numbers) |
| [bernoulli_numbers.hpp](../../combinatorics/bernoulli_numbers.hpp) | [Bernoulli数](bernoulli_numbers.md) | [図解](../../visualizations/b_12_15/index.html#bernoulli_numbers) |
| [narayana.hpp](../../combinatorics/narayana.hpp) | [Narayana数](narayana.md) | [図解](../../visualizations/b_12_15/index.html#narayana) |
| [lagrange_inversion.hpp](../../combinatorics/lagrange_inversion.hpp) | [Lagrange Inversion](lagrange_inversion.md) | [図解](../../visualizations/b_12_15/index.html#lagrange_inversion) |
| [stirling_transform.hpp](../../combinatorics/stirling_transform.hpp) | [Stirling Transform](stirling_transform.md) | [図解](../../visualizations/b_12_15/index.html#stirling_transform) |
| [binomial_transform.hpp](../../combinatorics/binomial_transform.hpp) | [二項変換](binomial_transform.md) | [図解](../../visualizations/b_12_15/index.html#binomial_transform) |
| [factorial_basis.hpp](../../combinatorics/factorial_basis.hpp) | [階乗べきとの基底変換](factorial_basis.md) | [図解](../../visualizations/b_12_15/index.html#factorial_basis) |
| [shift_sample_points.hpp](../../combinatorics/shift_sample_points.hpp) | [標本点シフト](shift_sample_points.md) | [図解](../../visualizations/b_12_15/index.html#shift_sample_points) |
| [binomial_prefix_sum.hpp](../../combinatorics/binomial_prefix_sum.hpp) | [二項係数のPrefix Sum](binomial_prefix_sum.md) | [図解](../../visualizations/b_12_15/index.html#binomial_prefix_sum) |

## N追加項目（2026-10-26）

[N全19項目の条件・検証](../number_theory_extensions/README.md) / [19モードの図解](../../visualizations/number_theory_extensions/index.html)。`make verify-number-theory-extensions`。

- [大引数・任意mod二項係数](prime_power_binomial.md)（N-06）
- [巨大なnの階乗 mod p](large_factorial.md)（N-07）

## C追加項目（2026-10-26）

[8項目の契約・検証](../combinatorics_extensions/README.md) / [図解](../../visualizations/combinatorics_extensions/index.html)。

- C-01 [q二項係数・q階乗](q_binomial.md)
- C-02 [上下境界付き単調列・格子路](bounded_monotone.md)
- C-03 [小素数法・巨大引数のStirling数](stirling_small_prime.md)
- C-04 [ラベルなし木の同型類数](unlabeled_trees.md)
- C-20 [RSK対応と逆対応](rsk.md)
- C-21 [Young図形と標準盤の列挙](young_tableaux.md)
- C-22 [Hook-Length / Hook-Content](hook_length.md)
- C-23 [Lah・Euler変換と逆変換](combinatorial_transforms.md)
