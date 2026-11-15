# JSON制約・CLI・差分テスト

[API](generation.md) / [設定例](../../examples/development/generation/arrays.json) / [検索カタログ](../../FEATURES.md)

以下は `ai_generated/` から実行します。

```sh
python3 tools/cplib.py gen --config examples/development/generation/arrays.json --seed 42
python3 tools/cplib.py gen --config examples/development/generation/arrays.json --cases 10 --out build/input.txt
python3 tools/cplib.py gen --config examples/development/generation/arrays.json --out-dir build/cases
python3 tools/cplib.py gen --config examples/development/generation/arrays.json --case-index 1
python3 tools/cplib.py gen --config examples/development/generation/arrays.json --validate
python3 tools/cplib.py gen --help
```

生成データだけをstdoutへ、診断をstderrへ出します。エラーは終了コード2です。
既存ファイルへの上書きには `--force` が必要です。設定ファイル自身への出力は禁止します。
`--out-dir` は `case-000000.txt` のような名前で、Tを付けず1ケース1ファイルを作ります。
`--case-index` は0始まりで、同一seed・設定・ケース数のバッチ中のケースを再現し、Tなしで出力します。
`--validate` はスキーマ検査に加えて指定seedで実際に全ケースを生成・整形します。他seedでの成功まで証明するものではありません。

## 最小設定

```json
{
  "seed": 42,
  "cases": {"count": 3, "format": "t_prefixed", "aggregate": {"N": {"sum_max": 20}}},
  "variables": [
    {"name": "N", "type": "int", "min": 1, "max": 10},
    {"name": "M", "type": "int", "min": 0, "max_ref": "N"},
    {"name": "K", "type": "int", "min": 1, "max": 1000000},
    {"name": "A", "type": "int_array", "length_ref": "N", "min": 1, "max": 1000000000}
  ],
  "output": [{"line": ["N", "M", "K"]}, {"line": ["A"]}]
}
```

variablesは宣言順に生成します。`max_ref: "N"`、`min_ref: "L"` などの `*_ref` は既出変数への式です。
数値フィールドには `"=N*(N-1)//2"` のような文字列式も使えます。
同じフィールドの直接値と `_ref` の両方はエラー。未知の型・キー・前方参照・重複変数名もエラーです。
式の整数演算は `+ - * // %`、比較は `< <= > >= == !=`、論理は `and or not`、
関数は `gcd / abs / min / max / len / sum` のみです。
evalは使用せずASTを検査します。属性アクセス・添字・任意関数・内包表記・べき乗は未対応です。

```json
{
  "cases": {"format": "plain"},
  "variables": [
    {"name": "N", "type": "int", "min": 2, "max": 100},
    {"name": "M", "type": "int", "min": 0, "max_ref": "N*(N-1)//2"},
    {"name": "L", "type": "int", "min": 1, "max_ref": "N"},
    {"name": "R", "type": "int", "min_ref": "L", "max_ref": "N"},
    {"name": "ABC", "type": "ordered_tuple", "length": 3, "min": 1, "max": 1000000000, "strict": true},
    {"name": "G", "type": "graph", "n_ref": "N", "m_ref": "M"}
  ],
  "output": [{"line": ["N", "M"]}, {"line": ["L", "R"]}, {"line": ["ABC"]}, {"edges": "G", "one_based": true}]
}
```

## 型とオプション

| type | 必須サイズ・値 | 主なオプション |
|---|---|---|
| int / float / fixed | — | min=0, max=100。int: mode,multiple,zero_weight。fixed: digits |
| bool / char | — | true_weight,false_weight / alphabet |
| choice / const | values / value | choiceに整数weights |
| tuple | ranges（`[[L,R], ...]`） | distinct |
| ordered_tuple | length | min,max,strict |
| interval / intervals | intervalsのみcount | min,max,half_open,length |
| int_array | length | min,max,unique,mode,distinct_count / total,sum_min,sum_max |
| string | length | alphabet,mode,unique,counts,distinct_count |
| brackets | pairs | mode |
| matrix | height,width | min,max,mode,ones,row_sums |
| grid | height,width | walls,connectivity,start_goal |
| tree / forest | n、forestのみcomponents | treeにmode |
| graph | n,m | directed,connected,components,loops,multi,weight_min,weight_max |
| dag / bipartite | n,m / left,right,m | — |
| queries | length,n | kind,min,max |
| polygon | n | bound（既存凸多角形） |

未指定の境界の既定値は一般に0〜100です。APIごとの制約と組合せ制限は [API説明](generation.md) を参照してください。
matrixのonesを使う場合は明示的に `min:0,max:1` を指定します。
graphにweight_min/maxを付けると `(u,v,w)` のレコード列を返し、通常のgraphは `Graph` を返します。
生成結果をPythonで扱う場合は `generate_cases(config)`、整形済み文字列なら `generate_text(config)` を使います。
出力layoutを変更しても乱数消費は変わりません。

## 複数変数の追加制約

トップレベルの `constraints: ["A != B", "A+B <= K", "gcd(A,B)==1"]` で生成後の条件を指定できます。
この場合はケース全体を最大 `attempts` 回（既定100）再生成します。予算切れは明確なエラーです。
構成的生成ができる条件は、順序タプル・依存min/max・配列のtotalを優先してください。
`A+B+C==S` のような希少条件を独立整数の棄却法に渡す代わりに、長さ3のtotal付き配列を生成します。
宣言途中で不可能な境界を作る設定は即時エラーです。任意の数式から実現可能な変数範囲を逆算する機能はありません。

## ケース数・総予算・再現性

casesの既定は `count=1, format="t_prefixed"`。EOF形式は `format="plain"`。
countは整数または `{"min":1,"max":10}` です。CLIの `--cases` はこの値を上書きします。
ランダムTではTのサンプリング後に予算の実現可能性を検査し、矛盾したTを自動調整しません。

```json
{
  "count": 10,
  "format": "t_prefixed",
  "aggregate": {
    "N": {"sum_max": 200000, "cost_max": 1000000000},
    "M": {"total": 1000}
  }
}
```

この例はcasesオブジェクトです。対象のNとMは **独立した非負整数** としてmin/maxを宣言してください。
依存するM、mode/multiple付き変数、配列そのものはaggregateの対象にできません。
`sum_min / sum_max / total` と、二乗和上限 `cost_max` をサポートします。
cost_maxと同時に使えるのはsum_maxだけです。予算は先に構成的に割り当てます。
`sum(N)` と `sum(M)` はそれぞれ守りますが、NとMの予算を結ぶ一般的な同時最適化は未対応です。

ケースごとにSplitMix64で導出した独立seedを用い、予算には別の乱数系列を使います。
1ケースの生成途中の乱数消費は他ケースに影響しません。aggregateの割当はTにも依存するため、
T・予算・変数構成を変更した場合に過去バッチのケースが保持される保証はありません。
個別再生成時は元バッチと同じ設定・T・seedを指定してください。

## 差分テスト

新ジェネレータを既存stressのgeneratorコマンドに登録します。seedの置換、失敗入力保存、リプレイは既存機構を使います。

```sh
mkdir -p build
g++ -std=c++17 -O2 -I . examples/development/fast.cpp -o build/generation-fast
python3 -m tools.cp_tools.stress --config examples/development/generation/stress.json --seed 100 --cases 80
# 失敗時に表示された保存ディレクトリを渡す
# python3 -m tools.cp_tools.stress --replay .stress-failures/seed-100-xxxxxxxx
```

[設定例](../../examples/development/generation/stress.json) は `gen --seed {seed}` を呼び出します。
stressのseedは外側のバッチseedです。1回に1入力を生成する設定では `count=1, format=plain` が便利です。
縮小器は未実装です。制約を壊す可能性がある行削除などは自動適用しません。

## 機能を探す・更新する

```sh
python3 tools/cplib.py search random --category test_generation
python3 tools/cplib.py search 文字列
python3 tools/cplib.py search shortest-path --status implemented
python3 tools/cplib.py catalog --update
python3 tools/cplib.py catalog --check
```

既存の `find` はヘッダ検索の動作を維持します。`python3 -m tools.cp_tools.catalog` は従来の検証台帳生成です。
新しい検索索引は [catalogの規約](../../catalog/README.md) に従います。
