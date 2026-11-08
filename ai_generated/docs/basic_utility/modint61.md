---
title: 2^61−1 modint
documentation_of: //basic_utility/modint61.hpp
---

# 2^61−1 modint

法`p=2^61−1`専用のmodintです。積を`low+(high)`へ折り畳み、`2^61 ≡ 1 (mod p)`を使って除算せずに剰余を計算します。

## 使用例

```cpp
#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/modint61.hpp"
int main() {
    using Mint = cp::ModInt61;
    std::string text = "ababa";
    Mint base = 911382323; // 固定値は再現用。hashの一致だけで文字列の同一性は保証しない。
    std::vector<Mint> prefix(text.size() + 1), power(text.size() + 1, Mint(1));
    for (std::size_t i = 0; i < text.size(); ++i) {
        prefix[i + 1] = prefix[i] * base + static_cast<unsigned char>(text[i]) + 1;
        power[i + 1] = power[i] * base;
    }
    auto hash = [&](std::size_t l, std::size_t r) { return prefix[r] - prefix[l] * power[r - l]; };
    std::cout << (hash(0, 3) == hash(2, 5)) << " " << (Mint(-1) * Mint(-1)) << "\n";
}
```

出力:

```text
1 1
```

## APIと使用条件

`ModInt61`の法は固定です。積`z`に対し`(z & p)+(z >> 61)`は`2p`未満なので、1回の減算で正規化できます。中間積はGCC/Clangの128bit整数です。符号付き／なし128bit整数からの構築も通常のmodで正規化します。

例は半開区間の多項式ハッシュに接続する方法を示します。固定baseは再現用で、敵対的入力への衝突保証はありません。ハッシュの一致は文字列の一致の証明にはなりません。既存[二重Rolling Hash](../string_algorithm/rolling_hash.md)のAPI・法は変更しません。例の前計算はO(n)時間／領域、区間hashはO(1)です。

## 共通modint API

`mod()` は法、`val()` は `[0,mod)` の代表元。既定値は0です。`bool`以外の組み込み整数（符号付き／なし128bitを含む）を正規化して構築できます。整数極小値も受け付けます。

四則演算・複合代入・単項正負・前置／後置`++`/`--`・`==`/`!=`、整数との混合演算に対応します。`pow(e)` は `0 <= e <= UINT64_MAX`。負の指数は`invalid_argument`、上限超過は`out_of_range`。`0^0=1 mod m`とします。

`try_inv()` は `optional<Mint>` を返し、逆元がなければ`nullopt`。`inv()`と逆元のない値による除算は`domain_error`です。素数法は要求せず、`gcd(x,m)=1`を確認します。法1では代表元0を逆元とし、すべての演算は0を返します。

`<<`は代表元、`>>`は符号付き10進整数トークンを扱います。桁数に上限はなく逐次剰余を計算します。不正なトークンは`failbit`を立て、対象値を変更しません。比較は同じ型の代表元の一致です。異なる法／IDの型同士の演算・暗黙変換は提供しません。`raw`など内部表現を直接作るAPIはありません。

共通の逆元計算には既存の[mod_inverse](../number_theory_basic/mod_inverse.md)を使います。整数演算を単位時間とすれば加減乗算・比較はO(1)、冪はO(log(e+1))、逆元・除算はO(log m)、入力は桁数Lに対してO(L)時間／トークン保持O(L)領域です。値1つの領域はO(1)。

## 使用する問題と検証

文字列・列のハッシュ、法2^61−1のDP。C++演算とPython任意精度の剰余を比較し、使用例で同じ部分文字列のハッシュを確認します。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#modint61) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
