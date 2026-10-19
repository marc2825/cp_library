---
title: ビット演算・固定popcount列挙
documentation_of: //basic_utility/bit_operations.hpp
---

# ビット演算・固定popcount列挙（F-14）

uint64のビット操作と、固定個数の1を持つmaskの遅延列挙を公開します。GCC/Clangのbuiltinを使用します。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/bit_operations.hpp"
int main() {
    std::cout << cp::topbit(0) << " " << cp::topbit(40) << " " << cp::lowbit(40) << "\n";
    bool first = true;
    for (auto mask : cp::FixedPopcountMasks(5, 2)) {
        if (!first) std::cout << " ";
        first = false; std::cout << mask;
    }
    std::cout << "\n";
}
```

出力:

```text
-1 5 3
3 5 6 9 10 12 17 18 20 24
```

## API・条件

| API | 規約 |
|---|---|
| `topbit(uint64_t)` | 最上位の1の位置、0なら-1 |
| `lowbit(uint64_t)` | 最下位の1の位置、0なら-1 |
| `popcount(uint64_t)` | 1の個数、0なら0 |
| `next_same_popcount(uint64_t)` | 同じpopcountの次の大きなuint64。存在しなければnullopt |
| `FixedPopcountMasks(int n,int k)` | 下位nビットからk個を選ぶmaskを数値の昇順で列挙 |

位置は0始まりであり、lowbitはmaskそのものを返しません。signed入力はuint64への変換を受けるため、符号を含む意味を期待せずuint64を渡してください。列挙条件は0≤k≤n≤64、違反はinvalid_argument。k=0は0を1回、n=k=64はUINT64_MAXを1回返します。空集合のn=k=0も0を1回です。

Gosperの次mask計算は最下位の1を繰り上げ、下側の1を詰め直します。0とuint64の桁あふれを事前・事後に処理し、64bitシフトや0除算を避けます。範囲内の列挙はnの上限も確認します。

個別演算・iteratorの1歩は64bit固定幅でO(1)、領域O(1)。総列挙数は二項係数C(n,k)です。全要素を配列化せずrange-forで使え、途中でbreakできます。n=64,k=32等は列挙数自体が非常に大きいため入力規模に応じて選んでください。

## 使用する問題・検証

固定個数選択、部分集合DP、bitsetの境界処理。16bit全入力の位置・個数、n≤16の全popcountクラス、64bitの端・k=0/1/63/64を直接列挙と照合します。PythonではGosperの式とは別に、繰り上げ可能な位置を探す方法で次maskを比較します。

`make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`でDebug/Release、単独include、全カテゴリ混在、説明内使用例、独立比較を検証します。オンラインACは未確認です。

[追加範囲・検証](../foundation_tools/README.md) / [一次資料](../foundation_tools/references.md) / [カテゴリ索引](README.md) / [図解](../../visualizations/foundation_tools/index.html#bit_operations) / [図の範囲](../../visualizations/foundation_tools/README.md)
