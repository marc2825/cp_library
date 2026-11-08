---
title: 実行時法modint
documentation_of: //basic_utility/dynamic_modint.hpp
---

# 実行時法modint

`DynamicModInt<Id=0>`（別名`dynamic_modint<Id>`）は実行時に法を変更できる型です。乗算には[Barrett Reduction](barrett_reduction.md)を共有します。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/dynamic_modint.hpp"
int main() {
    using A = cp::DynamicModInt<0>;
    using B = cp::DynamicModInt<1>;
    A::set_mod(12); B::set_mod(7);
    A a = 5;
    std::cout << a.pow(3) << " " << B(5).pow(3) << "\n";
    A::set_mod(11);
    try { std::cout << a.val(); }
    catch (const std::logic_error&) { std::cout << "stale\n"; }
}
```

出力:

```text
5 6
stale
```

## APIと使用条件

`set_mod(uint32_t m)` は `1 <= m <= UINT32_MAX`。初期法は998244353です。IDごとに法と世代番号を共有し、異なるIDは独立です。

**`set_mod`は同じ法を再設定しても既存値をすべて無効化します。** 古い値の`val()`・比較・算術・冪・逆元を使うと`logic_error`。コピーは世代も保持し、有効化しません。`x = Mint(整数)`で新しい値を代入できます。法の設定は値の構築より前に行ってください。無効な法は`invalid_argument`、世代番号上限は`overflow_error`で、失敗時は現在の法・既存値を保持します。

法と世代はC++17のinline staticで翻訳単位間でも共有します。同一IDの`set_mod`と他操作の並行実行には外部同期が必要です。世代を各値に持つため固定法版よりサイズと検査の負担があります。`set_mod`・世代検査はO(1)です。

## 共通modint API

`mod()` は法、`val()` は `[0,mod)` の代表元。既定値は0です。`bool`以外の組み込み整数（符号付き／なし128bitを含む）を正規化して構築できます。整数極小値も受け付けます。

四則演算・複合代入・単項正負・前置／後置`++`/`--`・`==`/`!=`、整数との混合演算に対応します。`pow(e)` は `0 <= e <= UINT64_MAX`。負の指数は`invalid_argument`、上限超過は`out_of_range`。`0^0=1 mod m`とします。

`try_inv()` は `optional<Mint>` を返し、逆元がなければ`nullopt`。`inv()`と逆元のない値による除算は`domain_error`です。素数法は要求せず、`gcd(x,m)=1`を確認します。法1では代表元0を逆元とし、すべての演算は0を返します。

`<<`は代表元、`>>`は符号付き10進整数トークンを扱います。桁数に上限はなく逐次剰余を計算します。不正なトークンは`failbit`を立て、対象値を変更しません。比較は同じ型の代表元の一致です。異なる法／IDの型同士の演算・暗黙変換は提供しません。`raw`など内部表現を直接作るAPIはありません。

共通の逆元計算には既存の[mod_inverse](../number_theory_basic/mod_inverse.md)を使います。整数演算を単位時間とすれば加減乗算・比較はO(1)、冪はO(log(e+1))、逆元・除算はO(log m)、入力は桁数Lに対してO(L)時間／トークン保持O(L)領域です。値1つの領域はO(1)。

## 使用する問題と検証

入力で法が与えられるDPや、複数法の独立計算。複数ID・同じ法への再設定・設定失敗・翻訳単位間の法共有を検証します。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#dynamic_modint) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
