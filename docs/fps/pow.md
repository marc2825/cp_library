---
title: FPS非負整数乗 pow
documentation_of: //fps/pow.hpp
---

# FPS非負整数乗 pow

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#pow) / [実装](../../fps/pow.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/pow.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_pow<17, 3>({0, 2, 1}, 3, 7));
}
```

[動く使用例](../../examples/fps/pow.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`fps_pow<Mod,Root>(a,uint64_t k,size_t n) -> vector<int>`

kは非負でuint64_t全域を扱います。負数を渡さないでください。n≤L。結果は正確にn項。n=0は空、k=0は入力によらず{1,0,…}で、0^0=1です。先頭の非0次数をvとすると、v*k≥nなら零列です。入力のn次以上は無視します。

次数v*kは整数として扱い、先に除算で上限を検査するため乗算超過を避けます。logの係数に掛けるk mod Modと、先頭係数の累乗a_v^kを区別します。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

a=x^v*c*u、u₀=1と分解し、a^k=x^(v*k)*c^k*exp(k log u)を計算します。先頭係数が1でない場合や先頭に0がある場合も使えます。

O(M(n)+log Mod log(n+1)+log(k+1))時間、O(n)補助領域。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: pow_of_formal_power_series](https://judge.yosupo.jp/problem/pow_of_formal_power_series)。[ジャッジ形式のプログラム](../../verify/fps/pow.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
