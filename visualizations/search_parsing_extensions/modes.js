(function(root){const modes=[
  {
    "id": "undo_manager",
    "title": "E-01 汎用Undo Manager・スコア検算",
    "doc": "search_parsing_extensions/undo_manager.md",
    "principle": "変更前の値や逆操作を記録し、履歴の末尾から逆順に戻す。スコア差分は全再計算と照合する。",
    "sample": {
      "values": [
        1,
        2
      ],
      "updates": [
        [
          0,
          3
        ],
        [
          1,
          4
        ]
      ],
      "rollback": true
    }
  },
  {
    "id": "index_set",
    "title": "E-02 IndexSet・一様ランダム選択",
    "doc": "search_parsing_extensions/index_set.md",
    "principle": "密な配列と逆位置を持ち、削除位置へ末尾の要素を移して穴を埋める。",
    "sample": {
      "universe": 6,
      "operations": [
        [
          "insert",
          2
        ],
        [
          "insert",
          4
        ],
        [
          "insert",
          1
        ],
        [
          "erase",
          4
        ]
      ]
    }
  },
  {
    "id": "tsp_neighborhood",
    "title": "E-03 TSPの2-opt・Or-opt",
    "doc": "search_parsing_extensions/tsp_neighborhood.md",
    "principle": "取り除く辺と加える辺だけで差分を求め、列の反転・回転で巡回路を変更する。",
    "sample": {
      "tour": [
        0,
        1,
        2,
        3
      ],
      "costs": [
        [
          0,
          8,
          4,
          4
        ],
        [
          8,
          0,
          4,
          4
        ],
        [
          4,
          4,
          0,
          8
        ],
        [
          4,
          4,
          8,
          0
        ]
      ],
      "operation": "two_opt",
      "first": 1,
      "last": 3,
      "count": 1,
      "after": 3,
      "symmetric": true
    }
  },
  {
    "id": "fast_annealing_acceptance",
    "title": "E-04 SA受理判定の対数テーブル",
    "doc": "search_parsing_extensions/fast_annealing_acceptance.md",
    "principle": "均等な確率区間の中点について−log(u)を前計算し、loss<T×閾値で採否を決める。",
    "sample": {
      "bits": 4,
      "loss": 0.7,
      "temperature": 1
    }
  },
  {
    "id": "delta_beam_search",
    "title": "E-05 差分更新型Beam Search",
    "doc": "search_parsing_extensions/delta_beam_search.md",
    "principle": "候補木の共通prefixをDFSし、辺をapply/undoしながら状態を共有する。残す候補だけを次のfrontierへ進める。",
    "sample": {
      "initial": 0,
      "target": 5,
      "moves": [
        -1,
        2
      ],
      "width": 3,
      "depth": 4
    }
  },
  {
    "id": "chokudai_search",
    "title": "E-06 Chokudai Search",
    "doc": "search_parsing_extensions/chokudai_search.md",
    "principle": "深さ別queueを消さずに保持し、後の周回でも見送った候補から探索を続ける。",
    "sample": {
      "initial": 0,
      "target": 5,
      "moves": [
        -1,
        2
      ],
      "depth": 4,
      "per_depth": 1,
      "max_expansions": 100,
      "max_nodes": 1000
    }
  },
  {
    "id": "parallel_search",
    "title": "E-20 多点並列探索・並列Multi-Start SA",
    "doc": "search_parsing_extensions/parallel_search.md",
    "principle": "開始点ごとに乱数系列を固定して並列に探索し、終了後にrun番号順で最良解を選ぶ。",
    "sample": {
      "starts": 4,
      "workers": 2,
      "seed": 42,
      "draws": 3,
      "bound": 100,
      "target": 37
    }
  },
  {
    "id": "distribution_sampling",
    "title": "E-21 Alias法・重み付き抽出・連続分布",
    "doc": "search_parsing_extensions/distribution_sampling.md",
    "principle": "Alias列を大小の重みで埋める。非復元抽出は指数時計、連続分布は逆関数とBox–Muller変換を使う。",
    "sample": {
      "weights": [
        1,
        3,
        2
      ],
      "distribution": "alias",
      "seed": 7,
      "count": 8,
      "rate": 2,
      "lower": 1,
      "upper": 3,
      "mean": 0,
      "deviation": 1
    }
  },
  {
    "id": "expression_parser",
    "title": "X-01 再帰下降parser・構文木",
    "doc": "search_parsing_extensions/expression_parser.md",
    "principle": "優先順位に従って再帰下降し、子を先に格納した構文木を反復的に評価する。",
    "sample": {
      "expression": "-2^2 + 3*(x+1)",
      "variables": {
        "x": "4"
      }
    }
  },
  {
    "id": "automata",
    "title": "X-02 DFA/NFAの共通操作",
    "doc": "search_parsing_extensions/automata.md",
    "principle": "ε閉包の集合をDFA状態にし、受理と遷移先の同値類で分割を細かくして最小化する。",
    "sample": {
      "states": 3,
      "alphabet": 2,
      "start": 0,
      "accepting": [
        2
      ],
      "edges": [
        [
          0,
          0,
          0
        ],
        [
          0,
          1,
          0
        ],
        [
          0,
          0,
          1
        ],
        [
          1,
          1,
          2
        ],
        [
          2,
          0,
          2
        ],
        [
          2,
          1,
          2
        ]
      ],
      "words": [
        [],
        [
          0
        ],
        [
          0,
          1
        ],
        [
          1,
          0,
          0
        ],
        [
          1,
          1,
          0,
          1,
          1
        ]
      ]
    }
  },
  {
    "id": "regex_nfa",
    "title": "X-03 正規表現からNFA・Thompson構成",
    "doc": "search_parsing_extensions/regex_nfa.md",
    "principle": "連接・選択・繰返しをε辺で結び、入力を読むごとに到達可能な状態集合を進める。",
    "sample": {
      "pattern": "(ab|c)*d?",
      "words": [
        "",
        "abccabd",
        "ac",
        "d",
        "abc"
      ]
    }
  }
];if(typeof module==='object')module.exports=modes;else root.SearchParsingModes=modes;})(globalThis);
