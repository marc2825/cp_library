---
title: FFT（実数・複素数）
documentation_of: //convolution/fft.hpp
---

# FFT（実数・複素数）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/convolution/index.html#fft-complex) / [実装](../../convolution/fft.hpp) / [参考資料](references.md)

## usage

```cpp
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>
#include "convolution/fft.hpp"
int main() {
    auto spectrum = cp::fft_real(std::vector<double>{1, 2, 3, 4});
    cp::fft(spectrum, true);
    std::cout << std::fixed << std::setprecision(3);
    for (auto x : spectrum) std::cout << x.real() << ' ';
    std::cout << '\n';
    std::vector<std::complex<long double>> z{{1, 1}, {2, -1}};
    cp::fft(z);
    for (auto x : z) std::cout << x.real() << ',' << x.imag() << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/convolution/fft.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`fft(vector<complex<Real>>& a, bool inverse=false)`

`fft_real(const vector<Real>& a) -> vector<complex<Real>>`

Realはfloat/double/long double。非空の長さは2冪で、満たさなければinvalid_argumentです。空は空、1要素も可。自動0埋めはせず、入出力は自然順の**全n成分**です。fft_realは実数列を複素数へ変換して順変換し、半分に圧縮しません。逆変換には返された複素列へfft(spectrum,true)を使います。

順変換はA[k]=Σ a[j] exp(**+**2πijk/n)。逆は負の指数で、最後にnで割ります。順→逆で元の列へ戻る規約です。FFTWなどの負の指数を順変換とするAPIとは符号が異なります。

実部・虚部がNaN/Infならinvalid_argumentで変更前に拒否します。有限入力でも計算途中が非有限になればoverflow_errorです。この場合は途中までの更新が残り得ます。逆変換も正規化前の途中値が表現可能である必要があります。

結果は浮動小数点の近似値です。絶対誤差は列長・係数・型・環境に依存します。小さい虚部や−0が残り、整数への四捨五入の正しさも一般には保証しません。整数の正確な積にはconvolution_ll、剰余の積にはconvolutionを使います。

## 原理・計算量

NTTと同じビット反転とradix-2バタフライを使い、単位根はcosθ+i sinθで求めます。実数入力ではA[n−k]=conj(A[k])という共役対称性があります。fft_realでも共通の複素FFTを使い、実数専用のパッキング最適化は行いません。

fftはO(n log n)時間・O(1)補助領域。fft_realは変換結果を確保するためO(n)領域。2冪の条件に加え、配列を確保できるメモリが必要です。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/atc001/tasks/fft_c)：高速フーリエ変換。ラッパは問題の小さい非負整数係数で逆変換後に丸めます。これは任意の整数入力への丸め保証ではありません。

7ヘッダの単独／逆順／混在includeと複数翻訳単位、5使用例、5ジャッジ形式・360ケースを検証します。14,641組の三値配列、8組の法／根、直接の有限体DFT・複素DFT、400ランダム整数積・300複素数積、Python任意精度858組・2,838出力と比較します。空・1要素・2冪境界・符号付き／符号なし極値・CRTの符号・保守的上限・NaN/Inf・非有限の途中値も含みます。

2^20点NTT、10万×10万の整数／mod積、65,537×65,537の実数FFTも確認します。doubleの大きい実数例では全出力の絶対誤差を測定しますが、この試験の誤差を一般の保証値とはしません。

[検証入口](../../verify/run_convolution.py)：通常／ASan・UBSan、debug/release。オンラインACは未確認です。
