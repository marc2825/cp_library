---
title: 整数分割の列挙
documentation_of: //basic_utility/integer_partitions.hpp
---

# 整数分割の列挙（U-10）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#integer_partitions)

現在の分割の末尾の1をまとめ、最後に残った部分を1減らします。余りをその部分以下になるよう大きい順に分配すると次の分割になります。反復実装なので大きなnで早期停止しても再帰スタックを使いません。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/integer_partitions.hpp"
int main() {
    std::cout << cp::integer_partition_count(5) << '\n';
    int visited = 0;
    cp::enumerate_integer_partitions(5, [&](const std::vector<int>& parts) {
        for (std::size_t i = 0; i < parts.size(); ++i) std::cout << (i ? "+" : "") << parts[i];
        std::cout << '\n';
        return ++visited < 3;
    });
}
```

出力:

```text
7
5
4+1
3+2
```

## API・使用条件

`integer_partition_count(int n)` は正整数の和による分割数をBigIntで返します。

`enumerate_integer_partitions(n, visit)` は各分割を非増加の `vector<int>` で渡し、分割列の降順辞書順で列挙します。同じ部分の順列を重複して出しません。callbackはboolを返し、falseで即停止。関数の戻り値は完走ならtrue、停止要求を受けたらfalseです。

n=0の分割は空列1個、n<0は `invalid_argument`。callbackへ渡すconst参照は次の列挙で変化するので、保存にはコピーが必要です。部分数制限などはcallback側で選別します。

## 計算量

列挙はO(n)追加記憶・1出力あたり最悪O(n)の遅延（callback費用を除く）。全てを保持しません。個数はO(n²)BigInt加算・O(n)個のBigInt記憶。

## 使用する問題・検証

整数分割を全て試すDPや探索。独立した再帰生成と順序まで比較し、個数DPとも照合。n=10万で最初の3件だけ取り出す早期停止も検証します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
