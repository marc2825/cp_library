---
title: Zobrist Hash
documentation_of: //heuristic_search/zobrist_hash.hpp
---

# Zobrist Hash

位置と値に乱数を割り当て、盤面の1位置の変更をXORだけで更新します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#zobrist) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`ZobristHash(size_t positions, size_t values, uint64_t seed=0)`。各位置は0〜values−1の値を一つ持ち、0も普通の値として乱数を割り当てます。空きマスは必要なら値0で表してください。values>=1。positions=0も可。

`positions()` / `values()`、`token(position,value) -> uint64_t`、`hash(const vector<Int>& state) -> uint64_t`、`change(hash,position,old_value,new_value)`。stateは位置数と同じ長さで、bool以外のsize_t以下の整数型、値は非負の範囲内です。changeは元の値がold_valueであることと、同じseed/表のhashであることを呼出側が保証します。

seedとサイズが同じなら表と結果は再現します。既存RandomのSplitMix64を使います。不正な値・位置はout_of_range、長さ・values=0はinvalid_argument、表容量超過はlength_error。

64bitハッシュの衝突はあり、同じhashだけで同じ状態とは証明できません。正確な重複除去には元の状態をkeyにして等値比較も行います。

## Usage

```cpp
#include <iostream>
#include "heuristic_search/zobrist_hash.hpp"
int main() {
    cp::ZobristHash z(3, 4, 42);
    std::vector<int> board{0, 1, 2};
    const auto original = z.hash(board);
    const auto changed = z.change(original, 1, 1, 3);
    board[1] = 3;
    std::cout << (changed == z.hash(board)) << '\n';
    std::cout << (z.change(changed, 1, 3, 1) == original) << '\n';
}
```

出力:

```text
1
1
```

[コンパイルできる使用例](../../examples/heuristic_search/zobrist_hash.cpp)。

## 原理

盤面全体は各位置のtokenのXORです。1位置を変えたとき、oldのtokenをXORして取り消し、newのtokenをXORして加えます。同じ変更を逆に適用すると元へ戻ります。

## 計算量

構築はO(positions×values)時間・領域。hashはO(positions)、token/changeはO(1)。

## 使用する問題と検証

盤面探索・探索履歴の候補索引・差分状態管理。ハッシュ値の確率性をAC判定にせず、独立SplitMix64表と全盤面再計算を照合します。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
