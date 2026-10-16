---
title: mod累乗・安全なmod乗算
documentation_of: //number_theory_basic/mod_pow.hpp
---

# mod累乗・安全なmod乗算

繰り返し二乗法で累乗を求めます。指数の下位ビットを消費し、現在の桁に対応する冪を二乗して進めます。補助のmod乗算は、積が符号なし型に収まる場合は直接計算し、それ以外は加算と倍増で求めます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#pow) / [実装](../../number_theory_basic/mod_pow.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/mod_pow.hpp"
int main() {
    std::cout << cp::mod_pow(-2, 10, 1000) << '\n';
    std::cout << cp::mod_multiply(9000000000000000000LL, 9000000000000000000LL,
                                  9223372036854775807LL) << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/mod_pow.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`mod_pow(long long base, long long exponent, long long modulus) -> long long`

`mod_multiply(long long a, long long b, long long modulus) -> long long`

- `modulus>=1`、累乗は `exponent>=0`。違反は `invalid_argument` です。
- 底・乗数は負数や `LLONG_MIN` でも可。結果は `[0,modulus)` の代表元です。
- `0^0=1` と定義し、法1なら指数0も含め常に0です。
- 素数法の条件はありません。負の指数は逆元を求めてから、非負指数の累乗として扱ってください。
- 標準C++17だけを使い、`__int128` や浮動小数点への変換に依存しません。

mod乗算は最悪O(log(modulus+1))時間、累乗は最悪O(log(exponent+1) log(modulus+1))時間、いずれもO(1)空間です。特に `modulus<=2^32` では正規化後の積が符号なし64ビットに収まり、乗算O(1)・累乗O(log(exponent+1))です。加算時も `x>=m-y` で分岐し、先に `x+y` を計算しません。

## 使用する問題・検証範囲

[使用する問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_B)：AOJ NTL_1_B。

小さい法の全乗数・繰り返し乗算と比較し、Pythonの任意精度の積とpowで64ビット全域・巨大指数・法1を照合します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
