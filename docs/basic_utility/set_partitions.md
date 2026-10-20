---
title: 集合分割・Restricted Growth列挙
documentation_of: //basic_utility/set_partitions.hpp
---

# 集合分割・Restricted Growth列挙（U-11）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#set_partitions)

RGSの右端から増やせる位置を探し、その位置を1増やして後ろを0へ戻します。prefix最大値を保持すると各位置の許されるブロック番号が分かります。Bell数は第二種Stirling数の行を更新して足し合わせます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/set_partitions.hpp"
int main() {
    std::cout << cp::set_partition_count(3) << '\n';
    cp::enumerate_set_partitions(3, [](const std::vector<int>& rgs) {
        for (int block : rgs) std::cout << block;
        std::cout << '\n';
        return true;
    });
}
```

出力:

```text
5
000
001
010
011
012
```

## API・使用条件

`enumerate_set_partitions(n, visit)` は `[0,n)` の集合分割をRestricted Growth String（RGS）で辞書順列挙します。rgs[i]は要素iのブロック番号で、rgs[0]=0、rgs[i]≤1+max(rgs[0..i))。ブロックは最小要素の順に番号付けし、ラベルの付け替えによる重複を排除します。

`partition_blocks(rgs)` はブロックごとの昇順要素列へ変換し、不正なRGSを `invalid_argument` にします。`set_partition_count(n)` はBell数をBigIntで返します。

n=0は空RGS1個、負のnは例外。callbackのfalseで即停止しfalseを返し、完走ならtrue。受け取ったconst参照を保存する場合はコピーしてください。既存 [set_function/set_partition.hpp](../../set_function/set_partition.hpp) の重み付き分割数・SPSとは別の、具体的分割を列挙するAPIです。

## 計算量

列挙はO(n)追加記憶、1出力あたり最悪O(n)遅延。blocksへの変換はO(n)。個数はO(n²)BigInt加算・小整数乗算、O(n)個のBigInt記憶。

## 使用する問題・検証

要素のグループ分けを全探索する問題。既存ブロックへの挿入または新規ブロックを作る再帰解と比較し、RGSの一意性・列挙順・Bell数・ブロック変換を検査します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
