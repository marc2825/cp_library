---
title: 多項式除算・剰余
documentation_of: //fps/division.hpp
---

# 多項式除算・剰余

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#division) / [実装](../../fps/division.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/division.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    const auto [q, r] = cp::polynomial_divmod<17, 3>({1, 2, 3, 4}, {1, 1});
    print(q); print(r);
}
```

[動く使用例](../../examples/fps/division.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`polynomial_divmod<Mod,Root>(a,b) -> pair<vector<int>,vector<int>>`

戻り値は{商q,剰余r}で a=b*q+r、deg(r)<deg(b)。正規化してから末尾0を除き、零多項式は空です。bが零ならdomain_error。aが零なら両方空、deg(a)<deg(b)なら商は空、剰余はtrim(a)。定数除数、単位でない最高次係数、入力の末尾0も扱います。

`polynomial_quotient<Mod,Root>(a,b)` / `polynomial_remainder<Mod,Root>(a,b)` はそれぞれ商／剰余だけを返します。それぞれdivmodを実行するため、両方必要ならdivmodを一度呼びます。正規化・trim後の両入力の長さがL以下である必要があります。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

多項式の除算は、高次から商を決めます。大きい入力では係数順を反転し、商の長さをkとして rev(q)=rev(a)*inv(rev(b)) mod x^k。商を逆順へ戻し、a−b*qから剰余を得ます。FPSのa*inv(b)とは用途が違い、ここではb₀=0も許されます。

元入力の正規化を含めO(|a|+|b|+M(N)+log Mod)時間、O(|a|+|b|+N)領域。Nは有効長の最大です。除数長または商長が32以下なら筆算でO(N)です。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: division_of_polynomials](https://judge.yosupo.jp/problem/division_of_polynomials)。[ジャッジ形式のプログラム](../../verify/fps/division.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
