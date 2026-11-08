---
title: FPS平方根 sqrt
documentation_of: //fps/sqrt.hpp
---

# FPS平方根 sqrt

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#sqrt) / [実装](../../fps/sqrt.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/sqrt.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    const auto root = cp::fps_sqrt<17, 3>({0, 0, 4, 4, 1}, 6);
    if (root) print(*root);
    else std::cout << "no square root\n";
}
```

[動く使用例](../../examples/fps/sqrt.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`fps_sqrt<Mod,Root>(a,size_t n) -> optional<vector<int>>`

**奇素数法専用**。Mod=2はコンパイル時に拒否します。n≤L。存在しないときはnullopt、存在すればn項です。n=0は成功した空列です。先頭n項が零ならn個の0を返します。最初の非0項の次数vが奇数、またはその係数が平方非剰余ならnulloptです。

非0先頭係数の平方根は[1,Mod−1]で小さい方を選びます。先頭0がある場合、gの一部の高次係数はg² mod x^nから決まりません。その末尾v/2項は0を選びます。n次以上の入力は使いません。nulloptは代数的な不成立で、長さ超過のlength_errorとは区別します。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

a=x^v*uからx^(v/2)を分離します。u₀の平方根をTonelli–Shanksで求め、g←(g+u/g)/2 mod x^mで精度を倍増します。存在判定では平方剰余のEuler判定を使います。

O(M(n)+log Mod log(n+1)+T_sqrt(Mod))時間、O(n)補助領域。T_sqrtは定数項のTonelli–Shanksで、非平方剰余をzまで順に探す費用O(z log Mod)とO(log² Mod)回の剰余乗算を含みます。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: sqrt_of_formal_power_series](https://judge.yosupo.jp/problem/sqrt_of_formal_power_series)。[ジャッジ形式のプログラム](../../verify/fps/sqrt.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
