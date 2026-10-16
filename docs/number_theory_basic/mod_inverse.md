---
title: mod逆元
documentation_of: //number_theory_basic/mod_inverse.hpp
---

# mod逆元

拡張Euclidで `a*x+m*y=g` を求めます。`g=1` の場合に `x` を `[0,m)` へ正規化すると逆元になります。合成数の法でも使えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#inverse) / [実装](../../number_theory_basic/mod_inverse.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/mod_inverse.hpp"
int main() {
    if (auto inverse = cp::mod_inverse(-3, 10)) std::cout << *inverse << '\n';
    std::cout << (cp::mod_inverse(6, 10).has_value() ? "exists" : "none") << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/mod_inverse.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`mod_inverse(long long a, long long modulus) -> optional<long long>`

- `modulus>=1`。0以下は `invalid_argument`。
- `gcd(a,modulus)!=1` なら `nullopt`。存在する場合は `[0,modulus)` の代表元を返します。
- `a` は負数・最小整数でも可。法1はACLと同じく代表元0を返します（`1 mod 1=0`）。
- 素数法でも `a` が法の倍数なら逆元は存在しません。`a^(m-2)` は一般の合成数法へ適用できません。

O(log(modulus+1))時間、O(1)空間。最初の剰余の計算をO(1)とし、拡張Euclidの商の更新で求めます。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/abc186/tasks/abc186_e)：AtCoder ABC186 E — Throne。GCDで方程式を約してから逆元を使います。

小さい法で全代表元を走査して逆元の有無を比較し、合成数・非互いに素・負数・法1・最大法をPython powの逆元と照合します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
