---
title: Sparse FPS
documentation_of: //fps/sparse_fps.hpp
---

# Sparse FPS

非零項が少ない入力から、逆数・log・exp・非負整数冪のdenseな低次項を生成します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#sparse_fps) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`SparseFPS=vector<pair<size_t,int>>`。`sparse_fps_inv<Mod>(terms,n)`、`_log`、`_exp`、`_pow(terms,uint64 exponent,n)`。次数の重複・順序は任意。

n次以上は無視し重複は加算。invはa0≠0、logはa0=1、expはa0=0。log/expはn≤Mod、powは先頭次数を移動した残りの精度≤Modが必要。invは次数の除算なし。powは0^0=1、先頭次数×指数の超過を避けます。n=0は空。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/sparse_fps.hpp"

int main() {
    auto c = cp::sparse_fps_inv(cp::SparseFPS{{1, -1}, {0, 1}}, 5);
    std::cout << c[4] << "\n";
}
```

出力:

```text
1
```

[コンパイルして動く例](../../examples/fps/sparse_fps_b.cpp)。

## 計算量

非零項数kとしてO(nk+k log k)時間、O(n+k)領域。NTTとRootは不要。

## 使用する問題・検証範囲

[Exp of Formal Power Series (Sparse)](https://judge.yosupo.jp/problem/exp_of_formal_power_series_sparse)。

直接係数漸化式、dense版、繰り返し積、重複次数、Mod境界を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
