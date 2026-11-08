(function(root){const modes=[
  {
    "id": "det_mod",
    "title": "L-01 任意modの行列式",
    "doc": "linear_algebra/determinant_any_mod.md",
    "sample": {
      "a": [
        [
          2,
          4
        ],
        [
          6,
          8
        ]
      ],
      "modulus": "12"
    }
  },
  {
    "id": "det_integer",
    "title": "L-02 整数行列の厳密行列式",
    "doc": "linear_algebra/determinant_integer.md",
    "sample": {
      "a": [
        [
          "100000000000000000001",
          "100000000000000000002"
        ],
        [
          "100000000000000000000",
          "100000000000000000001"
        ]
      ]
    }
  },
  {
    "id": "semiring",
    "title": "L-03 半環行列",
    "doc": "linear_algebra/semiring_matrix.md",
    "sample": {
      "a": [
        [
          null,
          3
        ],
        [
          -1,
          null
        ]
      ],
      "exponent": 2,
      "operation": "min"
    }
  },
  {
    "id": "pfaffian",
    "title": "L-04 Pfaffian",
    "doc": "linear_algebra/pfaffian.md",
    "sample": {
      "a": [
        [
          0,
          2,
          3,
          5
        ],
        [
          -2,
          0,
          7,
          11
        ],
        [
          -3,
          -7,
          0,
          13
        ],
        [
          -5,
          -11,
          -13,
          0
        ]
      ],
      "prime": 101
    }
  },
  {
    "id": "permanent",
    "title": "L-05 Permanent",
    "doc": "linear_algebra/permanent.md",
    "sample": {
      "a": [
        [
          1,
          2,
          0
        ],
        [
          0,
          3,
          4
        ],
        [
          5,
          0,
          6
        ]
      ]
    }
  },
  {
    "id": "smith",
    "title": "L-06 Smith標準形・整数／合同線形系",
    "doc": "linear_algebra/smith_normal_form.md",
    "sample": {
      "a": [
        [
          2,
          4
        ],
        [
          4,
          14
        ]
      ],
      "b": [
        6,
        18
      ],
      "modulus": "6"
    }
  },
  {
    "id": "hermite",
    "title": "L-07 Hermite標準形",
    "doc": "linear_algebra/hermite_normal_form.md",
    "sample": {
      "a": [
        [
          4,
          6
        ],
        [
          2,
          8
        ]
      ]
    }
  },
  {
    "id": "gf2",
    "title": "L-08 GF(2)多項式",
    "doc": "linear_algebra/gf2_polynomial.md",
    "sample": {
      "a": "11",
      "b": "7"
    }
  },
  {
    "id": "extension",
    "title": "L-08 汎用拡大有限体",
    "doc": "linear_algebra/finite_extension_field.md",
    "sample": {
      "prime": 3,
      "modulus": [
        1,
        0,
        1
      ],
      "a": [
        1,
        1
      ],
      "b": [
        2,
        1
      ]
    }
  },
  {
    "id": "frobenius",
    "title": "L-20 Frobenius標準形",
    "doc": "linear_algebra/frobenius_normal_form.md",
    "sample": {
      "a": [
        [
          1,
          1
        ],
        [
          0,
          1
        ]
      ],
      "prime": 5
    }
  },
  {
    "id": "hafnian",
    "title": "L-21 Hafnian",
    "doc": "linear_algebra/hafnian.md",
    "sample": {
      "a": [
        [
          0,
          2,
          3,
          5
        ],
        [
          2,
          0,
          7,
          11
        ],
        [
          3,
          7,
          0,
          13
        ],
        [
          5,
          11,
          13,
          0
        ]
      ]
    }
  },
  {
    "id": "charpoly",
    "title": "L-22 高速経路付き特性多項式",
    "doc": "linear_algebra/characteristic_polynomial_fast.md",
    "sample": {
      "a": [
        [
          1,
          2
        ],
        [
          3,
          4
        ]
      ],
      "prime": 998244353
    }
  }
];if(typeof module!=="undefined")module.exports=modes;else root.LinearAlgebraExtensionModes=modes;})(typeof window!=="undefined"?window:globalThis);
