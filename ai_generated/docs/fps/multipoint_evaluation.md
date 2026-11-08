---
title: 多点評価
documentation_of: //fps/multipoint_evaluation.hpp
---

# 多点評価

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#multipoint) / [実装](../../fps/multipoint_evaluation.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/multipoint_evaluation.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::multipoint_evaluation<17, 3>({1, 2, 3}, {0, 1, 2}));
}
```

[動く使用例](../../examples/fps/multipoint_evaluation.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`multipoint_evaluation<Mod,Root>(a,xs) -> vector<int>`

各xs[i]でa(xs[i])を評価し、入力の点の順で返します。法で同じ点を含めても使えます。空の点列なら入力多項式を処理せず空、空の多項式なら点数個の0です。xs.size()<L、および正規化・trim後のaの長さ≤Lが必要です。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

葉にX−xs[i]を置く積木を作り、根での剰余から子の積による剰余へ降ります。a=qM+rでM(xs[i])=0ならa(xs[i])=r(xs[i])。葉の剰余は定数なので、それが評価値です。積木の再利用用クラスは内部実装で、公開APIは一括処理です。

nを多項式の有効長、mを点数とするとO(|a|+M(n)+M(m+1) log(m+1))時間、O(|a|+m log(m+1))領域。積木はO(m log m)個の係数を保持します。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: multipoint_evaluation](https://judge.yosupo.jp/problem/multipoint_evaluation)。[ジャッジ形式のプログラム](../../verify/fps/multipoint_evaluation.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
