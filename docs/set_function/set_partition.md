---
title: 集合分割DP
documentation_of: //set_function/set_partition.hpp
---

# 集合分割DP

各非空ブロックの重みの積を、全てのラベルなし集合分割にわたって足します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#set_partition) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`set_partition_sum<Mod>(weights)`→各集合の総和。`set_partition_by_blocks<Mod>(weights)`→[ブロック数][集合]。weights[0]は無視。

空集合には0ブロックの分割が1つ。個数別では最小要素を含むブロックを選ぶことで順序の重複を除きます。個数別の版は小さいn向けで、SPS合成と同じ高速性は主張しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../set_function/set_partition.hpp"

int main() {
    auto a = cp::set_partition_by_blocks(std::vector<int>(8, 1));
    std::cout << a[2][7] << "\n";
}
```

出力:

```text
3
```

[コンパイルして動く例](../../examples/set_function/set_partition_b.cpp)。

## 計算量

総和はSPS expのO(n²2^n)時間・O(n2^n)領域。個数別はO(n3^n)時間・O(n2^n)領域。

## 使用する問題・検証範囲

[Exp of Set Power Series](https://judge.yosupo.jp/problem/exp_of_set_power_series)、重み付きBell・Stirling型の分割数え上げ。

restricted-growth stringで独立に列挙した全分割と、ブロック数別の総和を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
