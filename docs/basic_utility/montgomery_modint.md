---
title: Montgomery Reduction・64bit modint
documentation_of: //basic_utility/montgomery_modint.hpp
---

# Montgomery Reduction・64bit modint

`MontgomeryReduction64`は実行時の奇数法でMontgomery表現を操作し、`MontgomeryModInt64<Mod>`は同じ演算を固定法のmodintとして提供します。通常の値`x`を`xR mod m`（`R=2^64`）で保持して、乗算の除算を避けます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/montgomery_modint.hpp"
int main() {
    using Mint = cp::MontgomeryModInt64<9223372036854775807ULL>;
    std::cout << Mint(-1) * Mint(-1) << " " << Mint(2).pow(60) << "\n";
    cp::MontgomeryReduction64 reducer(101);
    auto product = reducer.multiply_encoded(reducer.encode(12), reducer.encode(13));
    std::cout << reducer.decode(product) << "\n";
}
```

出力:

```text
1 1152921504606846976
55
```

## APIと使用条件

どちらも **奇数 `1 <= m < 2^63`** に対応します。64bit全域の法ではありません。符号付き入力や128bit入力はコンストラクタで正規化され、内部値は常に`[0,m)`です。

| API | 条件・結果 |
|---|---|
| `MontgomeryReduction64(uint64_t m)` | 範囲外・偶数法は`invalid_argument` |
| `mod()` | 法m |
| `encode(uint64_t x)` | uint64全域のxから`xR mod m` |
| `decode(uint64_t x)` | `x<m`を通常表現へ変換。範囲外は`out_of_range` |
| `multiply_encoded(a,b)` | `a,b<m`の表現同士の積。結果もMontgomery表現。範囲外は`out_of_range` |
| `reduce(__uint128_t t)` | `0<=t<mR`で`t/R mod m`。範囲外は`out_of_range` |

構築時にNewton反復で`-m^{-1} mod R`を求めます。`q=(t mod R)*(-m^{-1}) mod R`とし、`(t+qm)/R`を計算して法を1回引きます。`m<2^63`・`t<mR`より和は128bitに収まります。固定法modintでは乗算の前後で通常表現へ戻しません。初期化・構築・公開reducerの操作はいずれもO(1)時間／領域、GCC/Clangの128bit整数を使います。速度の優劣は法・処理系に依存し、ベンチマークでの優越は主張しません。

## 共通modint API

`mod()` は法、`val()` は `[0,mod)` の代表元。既定値は0です。`bool`以外の組み込み整数（符号付き／なし128bitを含む）を正規化して構築できます。整数極小値も受け付けます。

四則演算・複合代入・単項正負・前置／後置`++`/`--`・`==`/`!=`、整数との混合演算に対応します。`pow(e)` は `0 <= e <= UINT64_MAX`。負の指数は`invalid_argument`、上限超過は`out_of_range`。`0^0=1 mod m`とします。

`try_inv()` は `optional<Mint>` を返し、逆元がなければ`nullopt`。`inv()`と逆元のない値による除算は`domain_error`です。素数法は要求せず、`gcd(x,m)=1`を確認します。法1では代表元0を逆元とし、すべての演算は0を返します。

`<<`は代表元、`>>`は符号付き10進整数トークンを扱います。桁数に上限はなく逐次剰余を計算します。不正なトークンは`failbit`を立て、対象値を変更しません。比較は同じ型の代表元の一致です。異なる法／IDの型同士の演算・暗黙変換は提供しません。`raw`など内部表現を直接作るAPIはありません。

共通の逆元計算には既存の[mod_inverse](../number_theory_basic/mod_inverse.md)を使います。整数演算を単位時間とすれば加減乗算・比較はO(1)、冪はO(log(e+1))、逆元・除算はO(log m)、入力は桁数Lに対してO(L)時間／トークン保持O(L)領域です。値1つの領域はO(1)。

## 使用する問題と検証

大きな奇数法での反復乗算・累乗。最大許容法・法1・合成数・逆元なしを検証し、実行時reducerの結果はPython任意精度で照合します。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#montgomery_modint) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
