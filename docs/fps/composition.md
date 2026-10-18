---
title: FPS合成
documentation_of: //fps/composition.hpp
---

# FPS合成

有限多項式fをgへ代入した低次項を求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#composition) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`fps_compose<Mod,Root>(f,g,n)`→正確にn項。n≤fps_max_size。素数法・原始根。

baby-step/giant-stepでgの小さい冪を保存し、係数ブロックを合成します。g[0]≠0も可。この場合fの高次も低次に寄与するためf全体を使用。無限FPSの合成として解釈する場合はg[0]=0が必要。n=0は空。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/composition.hpp"

int main() {
    auto c = cp::fps_compose(std::vector<int>{1, 2, 3}, std::vector<int>{1, 1}, 3);
    std::cout << c[0] << " " << c[1] << " " << c[2] << "\n";
}
```

出力:

```text
6 8 3
```

[コンパイルして動く例](../../examples/fps/composition_b.cpp)。

## 計算量

d=|f|、M(n)を積の費用としてO(sqrt(d)M(n)+nd)、領域O(n sqrt(d)+d)。

## 使用する問題・検証範囲

[Composition of Formal Power Series](https://judge.yosupo.jp/problem/composition_of_formal_power_series)。計画のCにある高速合成とは計算量が異なります。

独立した冪の逐次加算と任意定数の有限多項式合成を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
