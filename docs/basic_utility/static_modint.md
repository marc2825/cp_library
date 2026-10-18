---
title: 固定法modint
documentation_of: //basic_utility/static_modint.hpp
---

# 固定法modint

`StaticModInt<Mod>`（別名`static_modint<Mod>`）はコンパイル時の32bit法による整数環です。`ModInt998244353`・`ModInt1000000007`も用意します。関連する検証用型やparser内部型を置換せず、公開部品として独立させました。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/static_modint.hpp"
int main() {
    using Mint = cp::StaticModInt<12>;
    std::cout << Mint(-5) << " " << Mint(5).inv() << " " << Mint(6).try_inv().has_value() << "\n";
}
```

出力:

```text
7 5 0
```

## APIと使用条件

`Mod` は `1 <= Mod <= UINT32_MAX`。加減算の中間値は64bit、乗算は64bitの積を剰余します。合成数や `2^31` 以上の法も利用できます。型の法が固定なので同じ型の値は常に組み合わせられます。

## 共通modint API

`mod()` は法、`val()` は `[0,mod)` の代表元。既定値は0です。`bool`以外の組み込み整数（符号付き／なし128bitを含む）を正規化して構築できます。整数極小値も受け付けます。

四則演算・複合代入・単項正負・前置／後置`++`/`--`・`==`/`!=`、整数との混合演算に対応します。`pow(e)` は `0 <= e <= UINT64_MAX`。負の指数は`invalid_argument`、上限超過は`out_of_range`。`0^0=1 mod m`とします。

`try_inv()` は `optional<Mint>` を返し、逆元がなければ`nullopt`。`inv()`と逆元のない値による除算は`domain_error`です。素数法は要求せず、`gcd(x,m)=1`を確認します。法1では代表元0を逆元とし、すべての演算は0を返します。

`<<`は代表元、`>>`は符号付き10進整数トークンを扱います。桁数に上限はなく逐次剰余を計算します。不正なトークンは`failbit`を立て、対象値を変更しません。比較は同じ型の代表元の一致です。異なる法／IDの型同士の演算・暗黙変換は提供しません。`raw`など内部表現を直接作るAPIはありません。

共通の逆元計算には既存の[mod_inverse](../number_theory_basic/mod_inverse.md)を使います。整数演算を単位時間とすれば加減乗算・比較はO(1)、冪はO(log(e+1))、逆元・除算はO(log m)、入力は桁数Lに対してO(L)時間／トークン保持O(L)領域です。値1つの領域はO(1)。

## 使用する問題と検証

法付きDP・組合せ計算・行列積。[AOJ NTL_1_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_B)のmod累乗ラッパをローカル検証します。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#static_modint) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
