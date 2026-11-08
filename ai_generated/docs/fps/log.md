---
title: FPS対数 log
documentation_of: //fps/log.hpp
---

# FPS対数 log

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#log) / [実装](../../fps/log.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/log.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_log<17, 3>({1, 1}, 5));
}
```

[動く使用例](../../examples/fps/log.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`fps_log<Mod,Root>(a,size_t n) -> vector<int>`

n>0ではa₀=1が必要です。空や他の定数項はdomain_error。n=0は空。結果の定数項は0、長さはn。n次以上の入力は無視します。n≤Lを要求し、これにより積分で割る1,…,n−1が法で非0になります。有限体の定数の対数は提供しません。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

log(a)=∫a′/a dx。微分、FPS逆数、積、積分の順に計算し、積分定数を0に固定します。積分前の積はn−1項へ切り詰めます。

O(M(n)+log Mod)時間、O(n)補助領域。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: log_of_formal_power_series](https://judge.yosupo.jp/problem/log_of_formal_power_series)。[ジャッジ形式のプログラム](../../verify/fps/log.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。
