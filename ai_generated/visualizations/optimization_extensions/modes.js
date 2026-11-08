(function(root){const modes=[
  {
    "id": "linear_program",
    "title": "O-01 二段階Simplex・線形計画",
    "doc": "optimization_extensions/linear_program.md",
    "principle": "人工変数の総和を最小化し、実行可能基底を得て元の目的を改善する。",
    "sample": {
      "A": [
        [
          1,
          1
        ],
        [
          1,
          0
        ],
        [
          0,
          1
        ]
      ],
      "b": [
        4,
        2,
        3
      ],
      "c": [
        3,
        2
      ],
      "max_pivots": 80
    }
  },
  {
    "id": "low_dimensional_lp",
    "title": "O-02 固定1〜3次元の厳密LP",
    "doc": "optimization_extensions/low_dimensional_lp.md",
    "principle": "仮想的な[-M,M]^dの頂点をu+Mvとして解き、Mの係数を優先して比較する。",
    "sample": {
      "A": [
        [
          1,
          1
        ],
        [
          -1,
          0
        ],
        [
          0,
          -1
        ]
      ],
      "b": [
        4,
        0,
        0
      ],
      "c": [
        2,
        1
      ],
      "max_bases": 100
    }
  },
  {
    "id": "matroid_union",
    "title": "O-03 Matroid Union・Partition",
    "doc": "optimization_extensions/matroid_union.md",
    "principle": "未割当要素からBFSで交換先を探し、増加路に沿って所属する集合を組み替える。",
    "sample": {
      "groups": [
        0,
        0,
        1,
        1,
        1
      ],
      "capacities": [
        [
          1,
          2
        ],
        [
          1,
          1
        ]
      ]
    }
  },
  {
    "id": "fractional_programming",
    "title": "O-04 分数計画・Dinkelbach・比率二分",
    "doc": "optimization_extensions/fractional_programming.md",
    "principle": "比率の問題をf−λgの最適化に変え、選んだ点の比率または区間の中点へλを更新する。",
    "sample": {
      "points": [
        [
          5,
          2
        ],
        [
          7,
          3
        ],
        [
          9,
          4
        ]
      ],
      "start": 0,
      "lower": 0,
      "upper": 5,
      "denominator_lower_bound": 2,
      "max_iterations": 40
    }
  },
  {
    "id": "weighted_matroid_intersection",
    "title": "O-20 重み付きマトロイド交差",
    "doc": "optimization_extensions/weighted_matroid_intersection.md",
    "principle": "追加要素に−w、削除要素に+wの費用を置き、費用・辺数の順で最短の増加路を選ぶ。",
    "sample": {
      "groups1": [
        0,
        0,
        1,
        1
      ],
      "groups2": [
        0,
        1,
        0,
        1
      ],
      "capacities1": [
        1,
        1
      ],
      "capacities2": [
        1,
        1
      ],
      "weights": [
        8,
        7,
        6,
        1
      ]
    }
  },
  {
    "id": "base_polyhedron",
    "title": "O-21 Base Polyhedron・Lovász拡張",
    "doc": "optimization_extensions/base_polyhedron.md",
    "principle": "重み順のprefixでFの増分を座標へ割り当てると、基多面体上の線形目的が最適になる。",
    "sample": {
      "sets": [
        [
          0,
          1
        ],
        [
          1,
          2
        ],
        [
          2,
          3
        ]
      ],
      "weights": [
        3,
        2,
        1
      ]
    }
  },
  {
    "id": "submodular_optimization",
    "title": "O-22 劣モジュラ最適化",
    "doc": "optimization_extensions/submodular_optimization.md",
    "principle": "Lovász拡張のepigraphをLPで下から近似し、重み順の貪欲基を制約として追加する。",
    "sample": {
      "modular": [
        2,
        -3,
        1
      ],
      "edges": [
        [
          0,
          1,
          1
        ],
        [
          1,
          2,
          1
        ]
      ],
      "max_iterations": 30
    }
  },
  {
    "id": "integer_program",
    "title": "O-23 有限上下限付き整数線形計画",
    "doc": "optimization_extensions/integer_program.md",
    "principle": "LP緩和の目的値を上界とし、分数の変数をfloor/ceilで分けて探索する。",
    "sample": {
      "A": [
        [
          1,
          1
        ]
      ],
      "b": [
        3.5
      ],
      "c": [
        3,
        2
      ],
      "lower": [
        0,
        0
      ],
      "upper": [
        4,
        4
      ],
      "max_nodes": 100
    }
  },
  {
    "id": "omega_rational",
    "title": "B-20 Q(ω)の超現実数・無限小",
    "doc": "optimization_extensions/omega_rational.md",
    "principle": "ωを全ての有理定数より大きい正数とし、既約有理関数の最高次係数で符号を決める。",
    "sample": {
      "numerator": [
        -1,
        0,
        1
      ],
      "denominator": [
        -1,
        1
      ]
    }
  }
];if(typeof module==="object")module.exports=modes;else root.OptimizationModes=modes;})(globalThis);
