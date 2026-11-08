---
title: LCM Convolution
documentation_of: //number_theory_convolution/lcm_convolution.hpp
---

# LCM Convolution

$$
c(k)=\sum_{\mathrm{lcm}(i,j)=k}a(i)b(j)
$$

約数ゼータ変換した2列を要素ごとに掛け、約数メビウス変換で戻します。変換後の積は「iもjもkの約数」、つまりLCMがkの約数である全ての組を集めています。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_convolution/index.html#lcm) / [実装](../../number_theory_convolution/lcm_convolution.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "number_theory_convolution/lcm_convolution.hpp"
int main() {
    std::vector<long long> a{0, 1, 2, 3, 4, 5, 6}, b{0, 6, 5, 4, 3, 2, 1};
    const auto c = cp::lcm_convolution(a, b);
    for (std::size_t i = 1; i < c.size(); ++i) std::cout << c[i] << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/number_theory_convolution/lcm_convolution.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`lcm_convolution(const std::vector<T>& a, const std::vector<T>& b) -> std::vector<T>`

- 正の添字1以上だけを入力に使い、a[0],b[0]は無視します。存在する場合のc[0]は `T(0)`。
- 出力長は `max(a.size(),b.size())`。短い入力は0で補います。
- 両入力が空なら空。一方が空なら、他方と同じ長さのゼロ列です。長さ1だけなら `{T(0)}`。
- 入力は変更しません。LCMが出力上限Nを超える組は格納しません。
- より大きい出力上限Mまで求める場合は、入力を先にM+1要素へゼロで拡張します。入力の全組に対する完全な結果の長さを自動確保するAPIではありません。

係数型：`T(0)`、コピー、`+=`、`-=`、`operator*`。加算は可換な群、乗算は加算について分配的であること。除算や逆元は不要です。

上限Nはintに収まる必要があり、超過は `length_error`。係数演算の法はアルゴリズム側で固定しません。標準整数型（boolを除く）は全ての途中の加算・減算・乗算が表現可能である必要があり、超過は `overflow_error` です。符号なし整数の折り返しも行いません。独自型の算術条件は利用側が管理します。例外時も入力の配列は変更しません。

## 計算量・注意点

素数篩を含めO(N log log(N+2))時間。追加空間は係数列O(N)と素数篩・素数列です。3回の変換で素数列を共有します。 係数の演算・コピーをO(1)とした評価です。大きい型の場合はその費用が掛かります。`number_theory_basic/prime_sieve.hpp`を明示的に再利用しています。

最終結果が型に入る場合でも、ゼータ変換やその要素積などの途中値があふれる場合があります。十分な大きさの型、または目的のmod型を選んでください。

## 使用する問題・検証範囲

[使用する問題](https://judge.yosupo.jp/problem/lcm_convolution)。LCMごとの重み・個数。結果の範囲を超えるLCMは捨てます。

三値列・小さい全組と独立した約数／倍数走査・μの定義・全添字対を比較します。符号付き・mod998244353・合成数法12・非可換な行列の乗算、空・長さ違い・添字0・出力の切り捨て、整数極値／超過も確認します。100万上限で全出力と更新回数を検証します。

[検証入口](../../verify/run_number_theory_convolution.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、3ジャッジ形式・288ケース。オンラインACは未確認です。
