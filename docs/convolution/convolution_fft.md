---
title: 実数・複素数の畳み込み
documentation_of: //convolution/convolution_fft.hpp
---

# 実数・複素数の畳み込み

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/convolution/index.html#convolution-fft) / [実装](../../convolution/convolution_fft.hpp) / [参考資料](references.md)

## usage

```cpp
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>
#include "convolution/convolution_fft.hpp"
int main() {
    std::cout << std::fixed << std::setprecision(3);
    for (auto x : cp::convolution_fft(std::vector<double>{0.5, -1}, std::vector<double>{2, 3}))
        std::cout << x << ' ';
    std::cout << '\n';
    using C = std::complex<double>;
    for (auto x : cp::convolution_fft(std::vector<C>{{1, 1}, {2, 0}}, std::vector<C>{{0, 1}}))
        std::cout << x.real() << ',' << x.imag() << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/convolution/convolution_fft.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`convolution_fft(const vector<Real>& a, const vector<Real>& b) -> vector<Real>`

`convolution_fft(const vector<complex<Real>>& a, const vector<complex<Real>>& b) -> vector<complex<Real>>`

Realはfloat/double/long doubleで、両入力は同じ型を使います。実数型は実部だけを返し、複素数型は実部・虚部の両方を返します。整数列を渡す場合は浮動小数点のvectorへ明示的に変換してください。

空を含む場合は空です。非空なら|a|+|b|−1個で、末尾の0も残します。必要な2冪長へ自動で0埋めし、入力を変更しません。サイズの和・切り上げの上限超過はlength_error、非有限入力はinvalid_argument、変換や各点積の非有限値はoverflow_errorです。空の場合はもう一方の値を検査しません。メモリ不足などの標準の確保例外もあり得ます。

丸め誤差を含む近似値を返します。型の精度を上げても任意の大きい整数を正確に扱えるわけではありません。相殺のある出力では真値が0に近くても絶対誤差が残るため、検証では絶対誤差と相対誤差を併用します。

## 原理・計算量

FFT(a)、FFT(b)、各点積、逆FFTを使い、結果長まで切ります。0埋めによって巡回畳み込みの折り返しを避けます。実数版は複素数版と同じ変換を使ってから実部を返します。

切り上げ後の長さnに対してO(n log n)時間・O(n)補助領域。精度条件は[FFT](fft.md)を参照してください。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/atc001/tasks/fft_c)：高速フーリエ変換。小さい非負整数の係数を実数畳み込みとして扱う例です。

7ヘッダの単独／逆順／混在includeと複数翻訳単位、5使用例、5ジャッジ形式・360ケースを検証します。14,641組の三値配列、8組の法／根、直接の有限体DFT・複素DFT、400ランダム整数積・300複素数積、Python任意精度858組・2,838出力と比較します。空・1要素・2冪境界・符号付き／符号なし極値・CRTの符号・保守的上限・NaN/Inf・非有限の途中値も含みます。

2^20点NTT、10万×10万の整数／mod積、65,537×65,537の実数FFTも確認します。doubleの大きい実数例では全出力の絶対誤差を測定しますが、この試験の誤差を一般の保証値とはしません。

[検証入口](../../verify/run_convolution.py)：通常／ASan・UBSan、debug/release。オンラインACは未確認です。
