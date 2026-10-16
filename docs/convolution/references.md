# 一次資料・設計判断

2026-10-16に次の実装・仕様を読みました。外部コードや説明文を転載せず、radix-2の原理から独立に実装しています。

| 一次資料 | 確認した箇所 | 今回の設計 |
|---|---|---|
| [ACL convolution仕様](https://atcoder.github.io/ac-library/master/document_ja/convolution.html)・[実装](https://github.com/atcoder/ac-library/blob/master/atcoder/convolution.hpp) | NTTの法と2冪容量、空入力、整数CRTの範囲 | 入出力は通常の整数vector。反復radix-2を明示し、法／原始根をコンパイル時検査。整数版は独自の絶対値十分条件と混合基数の符号判定 |
| [CP-Algorithms FFT](https://cp-algorithms.com/algebra/fft.html) | 正の指数のDFT、ビット反転、逆変換、0埋め、NTT | NTT/FFTの規約を統一。実数・複素数の型を維持し、有限性とサイズを検査 |
| [FFTW: 1d DFT](https://www.fftw.org/fftw3_doc/The-1d-Discrete-Fourier-Transform-_0028DFT_0029.html)・[実数DFT](https://www.fftw.org/fftw3_doc/The-1d-Real_002ddata-DFT.html) | 指数の符号と正規化、実数入力の共役対称性 | FFTWと順方向の符号が逆であることを明記。fft_realは全n成分を返す |
| [Library Checker convolution_mod](https://judge.yosupo.jp/problem/convolution_mod)・[公式問題仕様](https://github.com/yosupo06/library-checker-problems/blob/master/convolution/convolution_mod/task.md) | 998244353の多項式積の入出力 | NTT手動経路とconvolution関数の2ラッパを独立の直接積と比較 |
| [AtCoder practice2 F](https://atcoder.jp/contests/practice2/tasks/practice2_f)・[ATC001 C](https://atcoder.jp/contests/atc001/tasks/fft_c) | mod積と整数の積の使用例 | CRT・FFT直接・FFT畳み込みで同じ整数入出力を検証。FFTの丸めは問題の入力範囲に合わせる |

ACLは[CC0](https://github.com/atcoder/ac-library/blob/master/LICENSE)、CP-Algorithmsの文章はCC BY-SA 4.0です。今回はコード・原文の転載をしていません。外部実装は実行依存にもしていません。オンラインACは未確認です。

整数版は3法の積より小さい範囲で復元するだけでは、真の整数がその範囲にあると判定できません。そのため入力から計算する十分条件を要求します。これは「最終結果だけがlong long内なら可」というACLの契約とは異なります。FFTの誤差やCRTの多義性を成功扱いにする代替にはしていません。
