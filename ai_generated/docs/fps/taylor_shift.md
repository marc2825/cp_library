---
title: Taylor Shift（多項式の平行移動）
documentation_of: //fps/taylor_shift.hpp
---

# Taylor Shift（多項式の平行移動）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#taylor) / [実装](../../fps/taylor_shift.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/taylor_shift.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::taylor_shift<17, 3>({1, 2, 3}, 2));
}
```

[動く使用例](../../examples/fps/taylor_shift.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`taylor_shift<Mod,Root>(a,long long c) -> vector<int>`

有限多項式fのf(x+c)の係数を、入力と同じ長さで返します。空は空、末尾0は保持。cはlong long全域を受け取り法で正規化します。n=|a|≤Lかつn≤Modを要求します。n>Lはlength_error、階乗の逆元がない場合はdomain_errorです（現在の共通容量制限なら後者は自動的に満たします）。

一般の無限FPSに非0定数を代入する操作ではありません。高次の項が低次へ寄与するため、入力を先に切り詰めると結果が変わります。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

g_i=Σ_(j≥i)a_j*C(j,i)*c^(j−i)。a_j*j!を反転した列とc^k/k!の列を畳み込み、積のn−1−i番目に1/i!を掛けます。

O(M(n)+log Mod)時間、O(n)補助領域。階乗と逆階乗はこの関数内で作ります。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: polynomial_taylor_shift](https://judge.yosupo.jp/problem/polynomial_taylor_shift)。[ジャッジ形式のプログラム](../../verify/fps/taylor_shift.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
