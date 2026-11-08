---
title: 温度制御・採用率の較正
documentation_of: //heuristic_search/temperature_control.hpp
---

# 温度制御・採用率の較正

悪化幅の標本から温度を較正し、観測した悪化手の採用率で温度を調整します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#temperature_control) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`calibrate_annealing_temperature(vector<long double> positive_losses,target)`。`AcceptanceTemperatureController(initial,target=.3,gain=1,min=1e-12,max=1e12)`、`observe(worsening,accepted)`、`update/temperature/observations`。

較正はmean(exp(-loss/T))=targetを解きます。targetは(0,1)、lossは正の有限値。controllerは悪化手だけを集計し、採用率が目標より低ければ温度を上げます。改善・同点を採用率へ混ぜません。updateで窓をリセット、観測0なら変更なし。min/max内へclamp。既存のgeometric/linear scheduleは保持します。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../heuristic_search/temperature_control.hpp"

int main() {
    auto temperature = cp::calibrate_annealing_temperature({2, 2, 2}, 0.5L);
    std::cout << (temperature > 2 && temperature < 3) << "\n";
}
```

出力:

```text
1
```

[コンパイルして動く使用例](../../examples/heuristic_search/temperature_control_b.cpp)。

## 計算量

較正は160回のlog温度二分探索でO(160N)、controllerは観測・更新ともO(1)時間・領域。

## 使用する問題・検証

SAの初期温度較正、実行中の悪化手採用率の調整。

複数尺度で直接平均採用率と目標の一致、観測の選別、増減・無観測・clampを比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
