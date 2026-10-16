---
title: 約数メビウス変換
documentation_of: //number_theory_convolution/divisor_mobius_transform.hpp
---

# 約数メビウス変換

$$
f(n)=\sum_{d\mid n}\mu(n/d)F(d)
$$

約数ゼータ変換の各更新を、素数もkも逆順で減算して戻します。μ(n)の表を明示的に構築する必要はありません。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_convolution/index.html#divisor-mobius) / [実装](../../number_theory_convolution/divisor_mobius_transform.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "number_theory_convolution/divisor_mobius_transform.hpp"
int main() {
    std::vector<long long> f{0, 1, 2, 3, 4, 5, 6}; // id(n)=n
    cp::divisor_mobius_transform(f); // phi = mu * id
    for (std::size_t i = 1; i < f.size(); ++i) std::cout << f[i] << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/number_theory_convolution/divisor_mobius_transform.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`divisor_mobius_transform(std::vector<T>& f) -> void`

- 正の添字1〜Nだけを変換します。Nは `f.size()-1`、添字0は読み書きせず保持します。
- 空ベクトルは何もしません。長さ1（添字0だけ）も保持します。
- in-placeで、逆変換は入力が特定のゼータ変換の結果であることを要求しません。任意の列へ定義式を適用します。

係数型：可換な加法群。`T& operator-=(const T&)` が必要です。整数では負の係数も現れるため、符号付き型やmod型を使うのが基本です。

上限Nはintに収まる必要があり、超過は `length_error`。係数演算の法はアルゴリズム側で固定しません。標準整数型（boolを除く）は全ての途中の加算・減算・乗算が表現可能である必要があり、超過は `overflow_error` です。符号なし整数の折り返しも行いません。独自型の算術条件は利用側が管理します。変換中の例外では、それ以前の更新が残るため、同じ配列で再開しないでください。

## 計算量・注意点

素数篩を含めO(N log log(N+2))時間。追加空間は素数篩O(N)ビットと素数列O(π(N))語です。 係数の演算・コピーをO(1)とした評価です。大きい型の場合はその費用が掛かります。`number_theory_basic/prime_sieve.hpp`を明示的に再利用しています。

逆変換を同じ加算順で減算すると、素数の冪に沿う値が消えすぎる場合があります。実装は素数と各kの走査を逆順にしています。

## 使用する問題・検証範囲

[使用する問題](https://judge.yosupo.jp/problem/lcm_convolution)。約数和から元の列を復元、id(n)=nからEulerのφ、LCM畳み込み。

三値列・小さい全組と独立した約数／倍数走査・μの定義・全添字対を比較します。符号付き・mod998244353・合成数法12・非可換な行列の乗算、空・長さ違い・添字0・出力の切り捨て、整数極値／超過も確認します。100万上限で全出力と更新回数を検証します。

[検証入口](../../verify/run_number_theory_convolution.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、3ジャッジ形式・288ケース。オンラインACは未確認です。
