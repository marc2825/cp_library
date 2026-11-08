---
title: Lah・Euler変換と逆変換
documentation_of: //combinatorics/combinatorial_transforms.hpp
---

# Lah・Euler変換と逆変換（C-23）

[ヘッダ](../../combinatorics/combinatorial_transforms.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#combinatorial_transforms)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/combinatorial_transforms.hpp"
int main() {
    const std::vector<int> a{0, 1, 1, 1, 1, 1};
    const auto lah = cp::lah_transform(a);
    const auto euler = cp::euler_transform(a);
    const auto original = cp::euler_transform(euler, true);
    for (const auto& row : {lah, euler, original}) {
        for (std::size_t i = 0; i < row.size(); ++i) std::cout << row[i] << (i + 1 == row.size() ? '\n' : ' ');
    }
}
```

出力:

```text
0 1 3 13 73 501
1 1 2 3 5 7
0 1 1 1 1 1
```

## API・条件

`lah_transform<Mod=998244353,Root=3>(vector<int> a,inverse=false)` は符号なしLah行列による変換。`euler_transform<Mod,Root>(a,inverse=false)` は積 `product_{k≥1}(1−x^k)^−a[k]` の係数への変換。ともに入力と同じ長さ、空入力は空です。各係数はModで正規化します。

Eulerの正方向はa[0]=0、逆方向はa[0]=1が必要（modで判定）で、違反は `invalid_argument`。Euler逆変換の出力先頭は0。指数係数は非負整数に限定せず、有限体上の形式的な二項級数として扱います。

素数Modと対応する原始根Rootをテンプレートで指定。長さn≤Mod、n≤`fps_max_size<Mod,Root>()` が必要です。容量超過は `length_error`、積分/逆階乗の標数条件違反は `domain_error`。既存の二項変換・Stirling変換とは別APIです。

## 仕組み

Lah数はn,k≥1で `L(n,k)=n!/k! * C(n−1,k−1)`、L(0,0)=1。`a[k]/(k!(k−1)!)` と `1/j!` の畳み込みにn!(n−1)!を掛けます。逆では後者を(−1)^j/j!へ替えます。

Eulerは `n*[x^n]log B = sum_{d|n} d*a[d]` を約数走査で作り、既存FPS expへ渡します。逆はFPS logを取り、確定したd*a[d]を真の倍数から引きます。

## 計算量

Lah O(M(n))時間・O(n)空間。Eulerは約数走査O(n log n)と既存FPS log/exp（O(M(n) log n)を上界とする処理）、O(n)作業空間。M(n)はNTT畳み込みのO(n log n)。

## 使用する問題

Lah数による順序付きリストへの分割、非ラベル多重集合の母関数、整数分割数のEuler積、積表示からの種類数の逆算。

## 検証

Lah漸化式の三角表による独立な行列積、正逆往復、Euler積を種類ごとのunbounded knapsackで比較。NTTが動く長さ65/96/128、負係数・空入力・不正定数項を確認。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
