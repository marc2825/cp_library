# 実行可能な生成例22項目

[API](../../../docs/development/generation.md) / [JSON](../../../docs/development/generation_json.md) / [機能検索](../../../FEATURES.md)

`ai_generated/` から `python3 -m examples.development.generation.demo 番号 --seed 42` を実行します。
出力はそのまま標準入力として使えます。下表の改行は `/` で示します。

| 番号 | 内容 | 標準入力形式 |
|---|---|---|
| 1 | 複数整数 | `N M` |
| 2 | 複数整数 | `N M K` |
| 3 | `1 <= L <= R <= N` | `N L R` |
| 4 | `A < B < C` | `A B C` |
| 5 | 整数配列 | `N / A1 ... AN` |
| 6 | 相異なる配列 | `N / A1 ... AN` |
| 7 | 総和300の配列 | `N / A1 ... AN` |
| 8 | 文字列 | `N / S` |
| 9 | 回文 | `N / S` |
| 10 | 正しい括弧列 | `N / S` |
| 11 | SからGへ到達可能なグリッド | `H W / H行の文字列` |
| 12 | 木 | `N M / M行のu v`（1-indexed） |
| 13 | 連結グラフ | `N M / M行のu v`（1-indexed） |
| 14 | DAG | `N M / M行のu v`（1-indexed） |
| 15 | 重み付きグラフ | `N M / M行のu v w`（頂点1-indexed） |
| 16 | 半開区間列 | `N Q / Q行のl r`（0-indexed） |
| 17 | 有効な集合操作列 | `N Q / Q行のinsert x または erase x` |
| 18 | 複数ケース | `T / N / A / N / A / ...` |
| 19 | 全ケースのsum(N)<=20 | `T / N / A / ...` |
| 20 | JSONから生成 | `T / N M K / A / ...` |

21. 既存の高速解・愚直解との差分テスト（0-indexedの `N M / u v`）:

```sh
mkdir -p build
g++ -std=c++17 -O2 -I . examples/development/fast.cpp -o build/generation-fast
python3 -m tools.cp_tools.stress --config examples/development/generation/stress.json --seed 100 --cases 80
```

22. 機能検索・カタログ更新:

```sh
python3 tools/cplib.py search 文字列 --category test_generation
python3 tools/cplib.py catalog --update
python3 tools/cplib.py catalog --check
```

追加のJSON例は [arrays.json](arrays.json)、[graph.json](graph.json)、[queries.json](queries.json) です。
queries.jsonは `N Q / A / 可変長の数値操作レコード` を出力します。
