---
title: Online FPS
documentation_of: //fps/online_fps.hpp
---

# Online FPS

入力が順に確定する状況で、逆数またはexpの同次数係数を返します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#online_fps) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`OnlineFPSInverse<Mod,Root>(terms)`、`OnlineFPSExp<Mod,Root>(terms)`、`append(int)->int`、`size()`。terms≤fps_max_size、expはterms≤Mod。

逆数はa0≠0、expはa0=0。最初のappendで検査し、拒否された先頭入力は訂正できます。以降はRelaxedConvolutionで既知部分の和を転送します。満杯はlength_error。途中のメモリ等の例外後の再利用は保証しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/online_fps.hpp"

int main() {
    cp::OnlineFPSExp<> series(4);
    series.append(0); series.append(1); series.append(0);
    std::cout << series.append(0) << "\n";
}
```

出力:

```text
166374059
```

[コンパイルして動く例](../../examples/fps/online_fps_b.cpp)。

## 計算量

N項を追加してO(N log²(N+1)+N log Mod)時間、O(N)領域。invの逆元費用は先頭1回、expは各次数。

## 使用する問題・検証範囲

オンラインな逆数・exp、[Exp of FPS](https://judge.yosupo.jp/problem/exp_of_formal_power_series)。

各appendの戻り値を独立な逆数・expの係数漸化式と比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
