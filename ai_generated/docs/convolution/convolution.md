---
title: 通常の畳み込み（mod）
documentation_of: //convolution/convolution.hpp
---

# 通常の畳み込み（mod）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/convolution/index.html#convolution) / [実装](../../convolution/convolution.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "convolution/convolution.hpp"
int main() {
    const std::vector<long long> a{1, -2, 3}, b{4, 5};
    for (int x : cp::convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
    for (int x : cp::convolution<17, 3>(a, b)) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/convolution/convolution.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`convolution<Mod=998244353,Root=3>(const vector<T>& a, const vector<U>& b) -> vector<int>`

T/Uはbool以外の64ビット以下の標準整数型です。異なる型や符号付き／符号なしの混在も可。負数、LLONG_MIN、ULLONG_MAXも正規化します。係数は定数項から昇冪順。入力は変更しません。

少なくとも一方が空なら空を返します。それ以外の長さは|a|+|b|−1で、末尾の0も残します。この長さを切り上げた2冪がNTTの上限内であることが必要です。満たさなければlength_error。和や切り上げ自体の桁あふれも避けます。

法と根の条件は[NTT](ntt.md)と同じです。任意の法への自動CRT変換は行いません。例えば法10^9+7の2冪容量は2であり、大きい入力には使えません。

## 原理・計算量

c[k]=Σ_(i+j=k) a[i]b[j]です。結果が折り返さないように|a|+|b|−1以上へ0埋めし、両列をNTT、各点の積、逆NTTの順で計算して必要な長さへ戻します。

これは通常の線形畳み込みで、添字をnで折り返す巡回畳み込みではありません。nを切り上げ後の長さとしてO(n log n)時間・O(n)補助領域。短い列にも同じNTT経路を使います。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/practice2/tasks/practice2_f)：Convolution。998244353の法で多項式を掛けます。

7ヘッダの単独／逆順／混在includeと複数翻訳単位、5使用例、5ジャッジ形式・360ケースを検証します。14,641組の三値配列、8組の法／根、直接の有限体DFT・複素DFT、400ランダム整数積・300複素数積、Python任意精度858組・2,838出力と比較します。空・1要素・2冪境界・符号付き／符号なし極値・CRTの符号・保守的上限・NaN/Inf・非有限の途中値も含みます。

2^20点NTT、10万×10万の整数／mod積、65,537×65,537の実数FFTも確認します。doubleの大きい実数例では全出力の絶対誤差を測定しますが、この試験の誤差を一般の保証値とはしません。

[検証入口](../../verify/run_convolution.py)：通常／ASan・UBSan、debug/release。オンラインACは未確認です。
