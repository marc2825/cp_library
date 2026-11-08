---
title: Dynamic Rolling Hash
documentation_of: //string_algorithm/dynamic_rolling_hash.hpp
---

# Dynamic Rolling Hash

点更新・挿入・削除・区間反転に対応する二重ハッシュです。既存ImplicitTreapの順方向・逆方向集約を共有します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#dynamic_rolling_hash) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`DynamicRollingHash(text={},base=RollingHash::default_base,seed=...)`、`insert/erase/set`、`reverse(l,r)`、`get(l,r)`、`equal(other,l,r,a,b)`、`lcp(other,l,r,a,b)`、`str/clear/size/base`。

byte+1、既存RollingHashと同じ法・base条件・Hash型。baseの異なる比較は拒否。各公開添字は例外で検査。Hashに長さやbaseは含まれません。衝突はあり、厳密な等値判定ではありません。削除したノードはclearまで保持し、累積挿入数<INT_MAX-1。問い合わせもTreapを分割・結合するため非constです。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/dynamic_rolling_hash.hpp"

int main() {
    cp::DynamicRollingHash text("abc");
    text.insert(1, 'x'); text.reverse(0, 4); text.set(0, 'z');
    std::cout << text.str() << "\n";
}
```

出力:

```text
zbxa
```

[コンパイルして動く使用例](../../examples/string_algorithm/dynamic_rolling_hash_b.cpp)。

## 計算量

構築O(N log N)期待、各編集と取得O(log N)期待、LCP O(log²N)期待、str O(N)。保存はclear以降の挿入回数Aに対してO(A)。

## 使用する問題・検証

動的文字列の部分列比較・回文候補検査。衝突を許容できる用途で使います。

7000回の挿入・削除・更新・反転をstd::stringと比較。各区間を既存静的ハッシュとも照合。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
