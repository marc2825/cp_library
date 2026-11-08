---
title: FPS指数 exp
documentation_of: //fps/exp.hpp
---

# FPS指数 exp

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#exp) / [実装](../../fps/exp.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/exp.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_exp<17, 3>({0, 1}, 5));
}
```

[動く使用例](../../examples/fps/exp.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`fps_exp<Mod,Root>(a,size_t n) -> vector<int>`

n>0ではa₀=0が必要で、違えばdomain_error。空は零級数として扱い、exp(0)={1,0,…}。n=0は定数項の条件を調べず空です。結果はn項で定数項1。n≤Lを要求します。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

log(g)=aを満たすgをNewton法で求めます。g₀=1から g←g(1+a−log(g)) mod x^m と更新し、精度を倍増します。有限体のlog/expは定数項の条件と、積分で必要な逆元が存在する範囲で使います。

O(M(n)+log Mod log(n+1))時間、O(n)補助領域。各段階でlogを計算し直しますが、幾何級数の長さの和からO(M(n))の畳み込みで収まります。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: exp_of_formal_power_series](https://judge.yosupo.jp/problem/exp_of_formal_power_series)。[ジャッジ形式のプログラム](../../verify/fps/exp.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
