---
title: FPS合成逆関数
documentation_of: //fps/compositional_inverse.hpp
---

# FPS合成逆関数

f(g(x))=xとなる級数gをNewton法で求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#compositional_inverse) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`fps_compositional_inverse<Mod,Root>(f,n)`。n>0ではf[0]=0,f[1]≠0。n≤fps_max_size。

g←g−(f(g)−x)/f'(g)で精度を倍にします。n=0では条件を検査せず空。微分係数がunitであるため小標数でも代数的に定義されますがNTT容量は既存制限を受けます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/compositional_inverse.hpp"

int main() {
    auto g = cp::fps_compositional_inverse(std::vector<int>{0, 1, 1}, 4);
    std::cout << g[1] << " " << g[2] << " " << g[3] << "\n";
}
```

出力:

```text
1 998244352 2
```

[コンパイルして動く例](../../examples/fps/compositional_inverse_b.cpp)。

## 計算量

合成の費用C(n)としてO(Σ C(2^j))時間、最大段の作業領域。今回の合成ではO(n²+n^(3/2)log(n+1))程度の上界。

## 使用する問題・検証範囲

[Compositional Inverse of Formal Power Series](https://judge.yosupo.jp/problem/compositional_inverse_of_formal_power_series)。

左右両方の合成がxに戻ること、非可逆条件を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
