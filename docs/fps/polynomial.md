---
title: 多項式の基本演算・微積分・切り詰め
documentation_of: //fps/polynomial.hpp
---

# 多項式の基本演算・微積分・切り詰め

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#polynomial) / [実装](../../fps/polynomial.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/polynomial.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    const std::vector<int> a{1, 2, 3}, b{-1, 1};
    print(cp::polynomial_add<17>(a, b));
    print(cp::polynomial_subtract<17>(a, b));
    print(cp::polynomial_multiply<17, 3>(a, b));
    print(cp::polynomial_derivative<17>(a));
    print(cp::polynomial_integral<17>(a));
    print(cp::fps_truncate<17>(a, 5));
}
```

[動く使用例](../../examples/fps/polynomial.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`polynomial_add<Mod>(a,b)` / `polynomial_subtract<Mod>(a,b)` は長さmax(|a|,|b|)で、足りない項を0として加減算します。

`polynomial_multiply<Mod,Root>(a,b)` は通常の積で、空を含めば空、非空は|a|+|b|−1項です。末尾0も保持します。全結果長を次の2冪へ切り上げた値が `ntt_max_size<Mod,Root>()` 以下であることが必要です。小さい片側（32項以下）は直接積で計算しますが、容量の条件は同じです。

`polynomial_derivative<Mod>(a)` は長さmax(0,|a|−1)。`polynomial_integral<Mod>(a)` は定数項0、長さ|a|+1。積分では |a|<Mod が必要で、満たさなければdomain_errorです。空の微分は空、空の積分は{0}です。

`fps_truncate<Mod>(a,n)` はa mod x^nを**正確にn項**で返し、不足分を0で補います。n=0は空、長さがvectorの表現上限を超えるとlength_errorです。NTTの容量制限はありません。

`polynomial_trim<Mod>(a)` は正規化後の末尾0を除去し、零多項式を空にします。`polynomial_evaluate<Mod>(a,long long x)` はHorner法による一点評価で、空は0です。xはlong long全域を受け取ります。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

乗算の係数は c[k]=Σ_(i+j=k)a[i]b[j]。微分は次数を掛けて1つ左へ、積分は次数の逆元を掛けて1つ右へ移します。積分の逆元列は線形時間の漸化式で作ります。

加減算・微分・積分・trim・一点評価はO(N)時間、返す配列はO(N)領域。一点評価の補助領域はO(1)。切り詰め・0補完はO(n)。積はO(M(|a|+|b|))時間、O(|a|+|b|)領域です。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: convolution_mod](https://judge.yosupo.jp/problem/convolution_mod)。基本操作のうち積のラッパは3.16の検証を参照し、加減算・微積分・切り詰めは独立した直接計算で検証します。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
