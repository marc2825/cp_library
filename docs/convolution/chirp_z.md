---
title: Chirp-Z Transform
documentation_of: //convolution/chirp_z.hpp
---

# Chirp-Z Transform

多項式を等比数列の点start*ratio^kで評価します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#chirp_z) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`chirp_z<Mod,Root>(f,start,ratio,count)`→count個。start,ratioはint。ratio=0なら最初はstart、以後は0。count=0は空、空多項式は零列。

ratio≠0では三角数T(i)=i(i-1)/2とij=T(i+j)-T(i)-T(j)を使います。逆元はratioだけで、平方根は不要。全長積の作業容量2N+M-2がNTT上限内で必要。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/chirp_z.hpp"

int main() {
    auto c = cp::chirp_z(std::vector<int>{1, 2, 1}, 1, 2, 3);
    std::cout << c[0] << " " << c[1] << " " << c[2] << "\n";
}
```

出力:

```text
4 9 25
```

[コンパイルして動く例](../../examples/convolution/chirp_z_b.cpp)。

## 計算量

|f|=N、count=M、切上げ2冪L≥2N+M-2としてO(L log L)時間・O(L)領域。

## 使用する問題・検証範囲

[Multipoint Evaluation on Geometric Sequence](https://judge.yosupo.jp/problem/multipoint_evaluation_on_geometric_sequence)。

直接Horner評価、start/ratio=0、負の入力、容量違反を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
