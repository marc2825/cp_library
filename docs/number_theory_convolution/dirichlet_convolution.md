---
title: Dirichlet Convolution
documentation_of: //number_theory_convolution/dirichlet_convolution.hpp
---

# Dirichlet Convolution

$$
c(n)=\sum_{ij=n}a(i)b(j)=\sum_{d\mid n}a(d)b(n/d)
$$

正整数の組(i,j)を `i*j<=N` の範囲で走査し、積a[i]*b[j]をc[i*j]へ加えます。通常の多項式畳み込みはi+j、こちらはi*jで出力先を決めます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_convolution/index.html#dirichlet) / [実装](../../number_theory_convolution/dirichlet_convolution.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "number_theory_convolution/dirichlet_convolution.hpp"
int main() {
    std::vector<long long> one(7, 1); one[0] = 0;
    const auto count = cp::dirichlet_convolution(one, one); // 約数個数 tau(n)
    for (std::size_t i = 1; i < count.size(); ++i) std::cout << count[i] << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/number_theory_convolution/dirichlet_convolution.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`dirichlet_convolution(const std::vector<T>& a, const std::vector<T>& b) -> std::vector<T>`

- 正の添字1以上だけを入力に使い、a[0],b[0]は無視します。存在する場合のc[0]は `T(0)`。
- 出力長は `max(a.size(),b.size())`。短い入力は0で補います。
- 両入力が空なら空。一方が空なら、他方と同じ長さのゼロ列です。長さ1だけなら `{T(0)}`。
- 入力は変更しません。添字の積i*jが出力上限Nを超える組は格納しません。
- より大きい出力上限Mまで求める場合は、入力を先にM+1要素へゼロで拡張します。入力の全組に対する完全な結果の長さを自動確保するAPIではありません。

係数型：`T(0)`、コピー、`+=`、`operator*` が必要です。加算は可換、乗算は加算について分配的であること。乗算の順序は常に `a[i]*b[j]` です。

上限Nはintに収まる必要があり、超過は `length_error`。係数演算の法はアルゴリズム側で固定しません。標準整数型（boolを除く）は全ての途中の加算・減算・乗算が表現可能である必要があり、超過は `overflow_error` です。符号なし整数の折り返しも行いません。独自型の算術条件は利用側が管理します。例外時も入力の配列は変更しません。

## 計算量・注意点

O(N log(N+1))時間、出力O(N)空間。総更新数は高々Σ⌊N/i⌋です。 係数の演算・コピーをO(1)とした評価です。大きい型の場合はその費用が掛かります。Dirichlet畳み込み自体は素数篩を呼びません。

出力係数は順序付きの組(i,j)から集計します。係数の乗算が可換なら、入力の交換についても可換になります。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/abc179/tasks/abc179_c)。1*1で約数個数、id*1で約数和、算術関数の積。一般の入力が乗法的関数である必要はありません。

三値列・小さい全組と独立した約数／倍数走査・μの定義・全添字対を比較します。符号付き・mod998244353・合成数法12・非可換な行列の乗算、空・長さ違い・添字0・出力の切り捨て、整数極値／超過も確認します。100万上限で全出力と更新回数を検証します。

[検証入口](../../verify/run_number_theory_convolution.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、3ジャッジ形式・288ケース。オンラインACは未確認です。
