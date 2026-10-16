---
title: 約数ゼータ変換
documentation_of: //number_theory_convolution/divisor_zeta_transform.hpp
---

# 約数ゼータ変換

$$
F(n)=\sum_{d\mid n}f(d)
$$

素数pごとに `f[k*p]+=f[k]` をkの小さい順で行います。同じ素数の冪に沿って値が伝わるので、2,4,8のような鎖でも約数を一度ずつ足せます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_convolution/index.html#divisor-zeta) / [実装](../../number_theory_convolution/divisor_zeta_transform.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "number_theory_convolution/divisor_zeta_transform.hpp"
int main() {
    std::vector<long long> f{99, 1, 2, 3, 4, 5, 6};
    cp::divisor_zeta_transform(f);
    for (std::size_t i = 1; i < f.size(); ++i) std::cout << f[i] << ' ';
    std::cout << '\n' << f[0] << '\n';
}
```

[動く使用例](../../examples/number_theory_convolution/divisor_zeta_transform.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`divisor_zeta_transform(std::vector<T>& f) -> void`

- 正の添字1〜Nだけを変換します。Nは `f.size()-1`、添字0は読み書きせず保持します。
- 空ベクトルは何もしません。長さ1（添字0だけ）も保持します。
- in-placeで、逆変換は入力が特定のゼータ変換の結果であることを要求しません。任意の列へ定義式を適用します。

係数型：加算が結合的・可換で、`T& operator+=(const T&)` を使えること。0を作る操作や減算・乗算は不要です。

上限Nはintに収まる必要があり、超過は `length_error`。係数演算の法はアルゴリズム側で固定しません。標準整数型（boolを除く）は全ての途中の加算・減算・乗算が表現可能である必要があり、超過は `overflow_error` です。符号なし整数の折り返しも行いません。独自型の算術条件は利用側が管理します。変換中の例外では、それ以前の更新が残るため、同じ配列で再開しないでください。

## 計算量・注意点

素数篩を含めO(N log log(N+2))時間。追加空間は素数篩O(N)ビットと素数列O(π(N))語です。 係数の演算・コピーをO(1)とした評価です。大きい型の場合はその費用が掛かります。`number_theory_basic/prime_sieve.hpp`を明示的に再利用しています。

素数ごとの更新には走査順の意味があります。全ての倍数を同じ向きで加えるO(N log N)の直接集計とは処理順が異なります。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/abc179/tasks/abc179_c)。約数和、約数個数、LCM畳み込み。

三値列・小さい全組と独立した約数／倍数走査・μの定義・全添字対を比較します。符号付き・mod998244353・合成数法12・非可換な行列の乗算、空・長さ違い・添字0・出力の切り捨て、整数極値／超過も確認します。100万上限で全出力と更新回数を検証します。

[検証入口](../../verify/run_number_theory_convolution.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、3ジャッジ形式・288ケース。オンラインACは未確認です。
