---
title: 正確な整数畳み込み
documentation_of: //convolution/convolution_ll.hpp
---

# 正確な整数畳み込み

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/convolution/index.html#integer) / [実装](../../convolution/convolution_ll.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "convolution/convolution_ll.hpp"
int main() {
    const std::vector<long long> a{1000000000, -2, 3}, b{4, 5};
    for (auto x : cp::convolution_ll(a, b)) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/convolution/convolution_ll.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`convolution_ll(const vector<long long>& a, const vector<long long>& b) -> vector<long long>`

64ビットlong longを使います。空を含む場合は空。非空の結果長は|a|+|b|−1で、2^24以下でなければlength_errorです。入力は変更せず、末尾の0も残します。

次の**十分条件**を事前に要求します。

B = min(Σ|a[i]| × max|b[j]|, Σ|b[j]| × max|a[i]|) ≤ 2^63

条件を満たさなければoverflow_errorです。全係数がlong longに収まる場合でも、打ち消しによってこの条件を満たさない入力は拒否することがあります。0列なら積は0なので大きい値と組み合わせても使えます。絶対値・和・積の検査自体は桁あふれしません。

復元後もlong longの範囲を確認し、正の2^63はoverflow_error、負の−2^63はLLONG_MINとして返します。FFTの丸めには依存しません。

## 原理・計算量

167772161、469762049、1224736769の3素数でNTT畳み込みを行い、CRTで整数を復元します。共通の2冪容量は2^24。3法の積Pは2^64より十分大きく、事前の絶対値上限があれば[−2^63,2^63]の候補を一意に選べます。

x=r₀+p₀*t₁+p₀*p₁*t₂という混合基数で、桁をP/2と比較して符号を決めます。Pを符号付き64ビットで直接作りません。uint64_tの規定された2^64を法とする演算で正の値／P−xの大きさを取り出します。__int128などの拡張型は不要です。

切り上げ後の長さnに対してO(n log n)時間・O(n)補助領域。3回の畳み込みを行う定数倍があり、大きい長さでは十分なメモリも必要です。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/atc001/tasks/fft_c)：高速フーリエ変換。整数の多項式の積を正確に求める用途にも使えます。

7ヘッダの単独／逆順／混在includeと複数翻訳単位、5使用例、5ジャッジ形式・360ケースを検証します。14,641組の三値配列、8組の法／根、直接の有限体DFT・複素DFT、400ランダム整数積・300複素数積、Python任意精度858組・2,838出力と比較します。空・1要素・2冪境界・符号付き／符号なし極値・CRTの符号・保守的上限・NaN/Inf・非有限の途中値も含みます。

2^20点NTT、10万×10万の整数／mod積、65,537×65,537の実数FFTも確認します。doubleの大きい実数例では全出力の絶対誤差を測定しますが、この試験の誤差を一般の保証値とはしません。

[検証入口](../../verify/run_convolution.py)：通常／ASan・UBSan、debug/release。オンラインACは未確認です。
