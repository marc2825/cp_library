---
title: NTT
documentation_of: //convolution/ntt.hpp
---

# NTT

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/convolution/index.html#ntt) / [実装](../../convolution/ntt.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "convolution/ntt.hpp"
int main() {
    std::vector<int> a{1, 2, 3, 4};
    cp::ntt<17, 3>(a);
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
    cp::ntt<17, 3>(a, true);
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/convolution/ntt.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`ntt<Mod=998244353,Root=3>(vector<int>& a, bool inverse=false)`

`ntt_max_size<Mod=998244353,Root=3>() -> size_t`（constexpr）

Modは2以上2×10^9以下の素数、Rootは乗法群の原始根です。どちらもコンパイル時に検査します。例は17/3、998244353/3、167772161/3、469762049/3、1224736769/3。Mod=2にはRoot=1を使います。

非空入力の長さnは2冪で、nがMod−1を割り切る必要があります。998244353の上限は2^23。非2冪はinvalid_argument、上限超過はlength_error、いずれも配列を書き換える前に拒否します。空は何もせず、1要素は剰余の正規化だけです。入力の負数・INT_MINも[0,Mod)へ正規化します。

順変換は **正の指数** です。w=Root^((Mod−1)/n)として、A[k]=Σ a[j]w^(jk)。逆変換はwの逆元を使い、最後にnの逆元を掛けます。入出力は自然な添字順です。

## 原理・計算量

ビット反転で偶数／奇数への分割順を並べ、長さ2,4,8,…のブロックを合成します。バタフライはu+w*vとu−w*v。複素数の単位根を有限体の単位根へ置き換え、整数演算だけで正確に計算します。

変換はO(n log n)時間、補助領域O(log Mod)（各法・根の共有キャッシュ）。最初の使用で単位根の表をO(log Mod)時間で作ります。パラメータのコンパイル時検査には素数判定とMod−1の試し割りを使います。呼び出しごとの動的配列は不要です。

## 使用する問題・検証範囲

[使用する問題](https://judge.yosupo.jp/problem/convolution_mod)：Convolution (Mod 998244353)。ラッパはNTTを直接呼び、0埋め・各点積・逆変換を行います。

7ヘッダの単独／逆順／混在includeと複数翻訳単位、5使用例、5ジャッジ形式・360ケースを検証します。14,641組の三値配列、8組の法／根、直接の有限体DFT・複素DFT、400ランダム整数積・300複素数積、Python任意精度858組・2,838出力と比較します。空・1要素・2冪境界・符号付き／符号なし極値・CRTの符号・保守的上限・NaN/Inf・非有限の途中値も含みます。

2^20点NTT、10万×10万の整数／mod積、65,537×65,537の実数FFTも確認します。doubleの大きい実数例では全出力の絶対誤差を測定しますが、この試験の誤差を一般の保証値とはしません。

[検証入口](../../verify/run_convolution.py)：通常／ASan・UBSan、debug/release。オンラインACは未確認です。
