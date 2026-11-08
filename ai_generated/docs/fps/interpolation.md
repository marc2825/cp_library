---
title: 多項式補間
documentation_of: //fps/interpolation.hpp
---

# 多項式補間

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#interpolation) / [実装](../../fps/interpolation.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/interpolation.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::polynomial_interpolation<17, 3>({0, 1, 2}, {1, 6, 0}));
}
```

[動く使用例](../../examples/fps/interpolation.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`polynomial_interpolation<Mod,Root>(xs,ys) -> vector<int>`

xsとysは同じ長さm。長さ違いはinvalid_argument。点は**法で相異なる**ことが必要で、重複はdomain_error（同じysでも拒否）です。m<L。結果は各点でf(xs[i])=ys[i]を満たす次数<mの多項式で、長さm、末尾0も保持します。空2列は空、1点の値0は{0}。点も値も負数を正規化します。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

M=∏(X−xs[i])と置くと、Lagrangeの重みはw_i=ys[i]/M′(xs[i])。積木の葉にw_iを置き、左の部分解には右の積、右には左の積を掛けて足します。根で求まる和が補間多項式です。

O(M(m+1) log(m+1)+log Mod)時間、O(m log(m+1))領域。M′(xs[i])の逆元はバッチ逆元で計算し、剰余累乗による逆元は一度だけです。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: polynomial_interpolation](https://judge.yosupo.jp/problem/polynomial_interpolation)。[ジャッジ形式のプログラム](../../verify/fps/interpolation.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
