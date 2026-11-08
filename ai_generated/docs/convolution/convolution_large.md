---
title: 巨大配列の畳み込み
documentation_of: //convolution/convolution_large.hpp
---

# 巨大配列の畳み込み

配列を分割し、各ブロック対の積を足すことで単一NTTの容量より長い結果を扱います。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#convolution_large) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`convolution_large<Mod,Root>(a,b,block_size=fps_max_size<Mod,Root>())`→vector<int>。block_sizeは正の2冪かつNTT容量の半分以下（容量1では1）。

結果を全てメモリに保持します。外部記憶方式ではありません。空を含むと空。不正ブロックinvalid_argument、結果長の超過length_error。変換結果のブロック間キャッシュはせず、同じ入力ブロックを繰り返し変換します。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/convolution_large.hpp"

int main() {
    auto c = cp::convolution_large<17, 3>(std::vector<int>(20, 1), std::vector<int>(20, 1), 8);
    std::cout << c.size() << " " << c[19] << "\n";
}
```

出力:

```text
39 3
```

[コンパイルして動く例](../../examples/convolution/convolution_large_b.cpp)。

## 計算量

ブロック長B、入力長N,MとしてO(ceil(N/B)ceil(M/B) B log(B+1)+N+M)時間、出力を含めO(N+M+B)領域。

## 使用する問題・検証範囲

[Convolution](https://judge.yosupo.jp/problem/convolution)。

法17のNTT容量16を超える長さの積、ランダム列を直接二重和と比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
