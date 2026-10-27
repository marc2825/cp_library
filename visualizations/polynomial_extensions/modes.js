(function(root){ const modes=[
  {
    "id": "extended_gcd",
    "title": "Q-01 多項式拡張GCD・剰余逆元",
    "doc": "fps/polynomial_extended_gcd.md",
    "sample": {
      "a": [
        2,
        -3,
        1
      ],
      "b": [
        -1,
        1
      ],
      "inverse": [
        0,
        1
      ],
      "modulus": [
        1,
        0,
        1
      ]
    }
  },
  {
    "id": "resultant",
    "title": "Q-02 終結式・判別式",
    "doc": "fps/polynomial_resultant.md",
    "sample": {
      "a": [
        2,
        -3,
        1
      ],
      "b": [
        -3,
        1
      ]
    }
  },
  {
    "id": "factorization",
    "title": "Q-03 有限体上の多項式因数分解",
    "doc": "fps/polynomial_factorization.md",
    "sample": {
      "a": [
        2,
        1,
        0,
        1,
        2
      ],
      "prime": 3
    }
  },
  {
    "id": "wrap_convolution",
    "title": "Q-04 mod 2^64 畳み込み",
    "doc": "convolution/convolution_u64.md",
    "sample": {
      "a": [
        "18446744073709551615",
        "2"
      ],
      "b": [
        "2",
        "3"
      ]
    }
  },
  {
    "id": "nimber_convolution",
    "title": "Q-05 GF(2^64) 畳み込み",
    "doc": "convolution/convolution_nimber.md",
    "sample": {
      "a": [
        "1",
        "2",
        "3"
      ],
      "b": [
        "2",
        "1"
      ]
    }
  },
  {
    "id": "transpose",
    "title": "Q-06 転置原理・固定畳み込みの共有前計算",
    "doc": "convolution/transposed_convolution.md",
    "sample": {
      "kernel": [
        1,
        2
      ],
      "input": [
        3,
        4,
        5
      ],
      "weights": [
        7,
        8,
        9
      ]
    }
  },
  {
    "id": "doubling",
    "title": "Q-07 NTT Doubling",
    "doc": "convolution/ntt_doubling.md",
    "sample": {
      "a": [
        1,
        2
      ],
      "prime": 17
    }
  },
  {
    "id": "subset_sum",
    "title": "Q-08 Sharp P Subset Sum",
    "doc": "fps/sharp_p_subset_sum.md",
    "sample": {
      "weights": [
        0,
        1,
        2,
        2
      ],
      "limit": 5
    }
  },
  {
    "id": "prefix_sum",
    "title": "Q-09 多項式の不定和分",
    "doc": "fps/polynomial_prefix_sum.md",
    "sample": {
      "a": [
        0,
        1
      ],
      "prime": 17
    }
  },
  {
    "id": "rader",
    "title": "Q-20 Rader FFT",
    "doc": "convolution/rader_fft.md",
    "sample": {
      "a": [
        [
          1,
          0
        ],
        [
          2,
          0
        ],
        [
          3,
          0
        ]
      ]
    }
  },
  {
    "id": "bluestein",
    "title": "Q-21 Bluestein FFT",
    "doc": "convolution/bluestein_fft.md",
    "sample": {
      "a": [
        [
          1,
          0
        ],
        [
          2,
          0
        ],
        [
          3,
          0
        ]
      ]
    }
  },
  {
    "id": "ntt64",
    "title": "Q-22 Goldilocks 64bit NTT",
    "doc": "convolution/ntt64.md",
    "sample": {
      "a": [
        "18446744069414584320",
        "2"
      ],
      "b": [
        "2",
        "3"
      ]
    }
  },
  {
    "id": "avx2",
    "title": "Q-23 AVX2 NTT",
    "doc": "convolution/ntt_avx2.md",
    "sample": {
      "a": [
        1,
        2,
        3,
        4
      ],
      "prime": 17
    }
  },
  {
    "id": "cyclic_tensor",
    "title": "Q-25 多変数巡回畳み込み",
    "doc": "convolution/multidimensional_cyclic_convolution.md",
    "sample": {
      "a": [
        1,
        2,
        3,
        4
      ],
      "b": [
        1,
        0,
        0,
        1
      ],
      "shape": [
        2,
        2
      ]
    }
  },
  {
    "id": "multiplicative",
    "title": "Q-26 乗法剰余添字の畳み込み",
    "doc": "convolution/multiplicative_convolution.md",
    "sample": {
      "a": [
        1,
        2,
        3,
        4,
        5
      ],
      "b": [
        0,
        1,
        0,
        0,
        0
      ]
    }
  },
  {
    "id": "projection",
    "title": "Q-27 Power Projection",
    "doc": "fps/power_projection.md",
    "sample": {
      "weights": [
        1,
        2,
        3,
        4
      ],
      "g": [
        0,
        1,
        1
      ],
      "count": 4
    }
  },
  {
    "id": "roots",
    "title": "Q-28 有限体の根の列挙",
    "doc": "fps/polynomial_roots.md",
    "sample": {
      "a": [
        15,
        5,
        13,
        1
      ],
      "prime": 17
    }
  },
  {
    "id": "composition",
    "title": "Q-29 高速FPS合成",
    "doc": "fps/fast_composition.md",
    "sample": {
      "outer": [
        1,
        2,
        3
      ],
      "inner": [
        0,
        1,
        2
      ],
      "n": 6
    }
  },
  {
    "id": "fractional",
    "title": "Q-30 FPSの一次分数変数変換",
    "doc": "fps/fractional_linear_transform.md",
    "sample": {
      "a": [
        1,
        2,
        3
      ],
      "c": 1,
      "n": 5
    }
  },
  {
    "id": "multivariate",
    "title": "Q-31 多変数FPS",
    "doc": "fps/multivariate_fps.md",
    "sample": {
      "a": [
        1,
        -1,
        -1,
        0
      ],
      "shape": [
        2,
        2
      ]
    }
  },
  {
    "id": "ode",
    "title": "Q-33 FPSの微分方程式",
    "doc": "fps/differential_equation.md",
    "sample": {
      "coefficients": [
        [],
        [],
        [
          1
        ]
      ],
      "initial": 1,
      "n": 6
    }
  },
  {
    "id": "polynomial_matrix",
    "title": "Q-24 多項式行列式・評価行列Prefix Product",
    "doc": "fps/polynomial_matrix.md",
    "sample": {
      "matrix": [
        [
          [
            0,
            1
          ],
          [
            1
          ]
        ],
        [
          [
            1
          ],
          [
            0,
            1
          ]
        ]
      ],
      "start": 0,
      "count": 3
    }
  },
  {
    "id": "composed",
    "title": "Q-34 Composed Sum / Product",
    "doc": "fps/composed_polynomial.md",
    "sample": {
      "a": [
        2,
        -3,
        1
      ],
      "b": [
        -3,
        1
      ]
    }
  },
  {
    "id": "middle",
    "title": "Q-35 高速中間積",
    "doc": "convolution/fast_middle_product.md",
    "sample": {
      "a": [
        1,
        2,
        3,
        4,
        5
      ],
      "b": [
        2,
        1
      ]
    }
  },
  {
    "id": "holonomic",
    "title": "Q-36 Holonomic数列の推定と大きな添字",
    "doc": "linear_recurrence/holonomic.md",
    "sample": {
      "initial": [
        1
      ],
      "p": [
        [
          -1,
          -1
        ],
        [
          1
        ]
      ],
      "n": 10
    }
  },
  {
    "id": "reeds_sloane",
    "title": "Q-37 Reeds–Sloane",
    "doc": "linear_recurrence/reeds_sloane.md",
    "sample": {
      "samples": [
        1,
        1,
        2,
        3,
        5,
        8,
        4,
        3
      ],
      "modulus": 9
    }
  },
  {
    "id": "labelled",
    "title": "Q-38 SET・SEQ・CYCの母関数合成",
    "doc": "linear_recurrence/combinatorial_composition.md",
    "sample": {
      "a": [
        0,
        1
      ],
      "n": 5,
      "operation": "sequence"
    }
  },
  {
    "id": "recurrence_slice",
    "title": "Q-39 線形漸化式の連続大量項",
    "doc": "linear_recurrence/recurrence_slice.md",
    "sample": {
      "initial": [
        0,
        1
      ],
      "coefficients": [
        1,
        1
      ],
      "start": 10,
      "count": 6
    }
  },
  {
    "id": "sps_projection",
    "title": "Q-40 SPS Power Projection",
    "doc": "set_function/sps_power_projection.md",
    "sample": {
      "weights": [
        1,
        1,
        1,
        1
      ],
      "a": [
        0,
        2,
        3,
        4
      ],
      "count": 5
    }
  },
  {
    "id": "online_subset",
    "title": "Q-41 オンライン集合関数畳み込み",
    "doc": "set_function/online_subset_convolution.md",
    "sample": {
      "a": [
        1,
        2,
        3,
        4
      ],
      "b": [
        1,
        1,
        1,
        1
      ]
    }
  },
  {
    "id": "intersection",
    "title": "Q-42 交差サイズ別の集合畳み込み",
    "doc": "set_function/intersection_convolution.md",
    "sample": {
      "a": [
        1,
        2,
        3,
        4
      ],
      "b": [
        1,
        1,
        1,
        1
      ]
    }
  }
]; if(typeof module!=="undefined")module.exports=modes;else root.PolynomialModes=modes;})(typeof window!=="undefined"?window:globalThis);
