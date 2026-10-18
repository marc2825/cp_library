---
title: Relaxed / Online Convolution
documentation_of: //convolution/relaxed_convolution.hpp
---

# Relaxed / Online Convolution

両列の係数を1項ずつ受け取り、その時点の積の係数を返します。前の出力を次の入力に使えるため、自己依存する級数にも使えます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#relaxed_convolution) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`RelaxedConvolution<Mod=998244353,Root=3>(terms)`、`append(int a_t,int b_t)->int`、`size()`、`capacity()`。容量は出力項数、0も可。素数Modと原始根Root、terms≤fps_max_sizeが必要。

入力は正規化し出力は[0,Mod)。満杯はlength_error。各組(i,j)をmin(i,j)の最高位bitごとに分け、完成したブロックと先頭側の帯の積を将来の係数へ予約します。0番目を含む項は即時計算します。例外発生時のオブジェクトの再利用は保証しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/relaxed_convolution.hpp"

int main() {
    cp::RelaxedConvolution<> product(3);
    std::cout << product.append(1, 4) << " " << product.append(2, 5) << " " << product.append(3, 6) << "\n";
}
```

出力:

```text
4 13 28
```

[コンパイルして動く例](../../examples/convolution/relaxed_convolution_b.cpp)。

## 計算量

N項をすべて追加してO(N log²(N+1))時間、O(N)領域。構築O(N)。単独appendの最悪はO(N log N)、償却時間と区別します。

## 使用する問題・検証範囲

[Convolution](https://judge.yosupo.jp/problem/convolution)、オンラインFPS exp / inv。

180項までの全prefixと32768項の全1列を直接和と比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
