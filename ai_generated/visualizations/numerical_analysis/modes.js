(function(root){const modes=[
  {
    "id": "quadratic",
    "title": "Y-01 安定な二次方程式",
    "doc": "numerical_analysis/quadratic.md",
    "sample": {
      "a": 1,
      "b": -5,
      "c": 6
    }
  },
  {
    "id": "bisection",
    "title": "Y-01 二分法による求根",
    "doc": "numerical_analysis/bisection.md",
    "sample": {
      "coefficients": [
        -2,
        0,
        1
      ],
      "lower": 0,
      "upper": 2,
      "start": 1,
      "tolerance": 1e-10,
      "max_iterations": 80
    }
  },
  {
    "id": "newton",
    "title": "Y-01 Newton法",
    "doc": "numerical_analysis/newton.md",
    "sample": {
      "coefficients": [
        -2,
        0,
        1
      ],
      "lower": 0,
      "upper": 2,
      "start": 1,
      "tolerance": 1e-10,
      "max_iterations": 80
    }
  },
  {
    "id": "brent",
    "title": "Y-01 Brent法",
    "doc": "numerical_analysis/brent.md",
    "sample": {
      "coefficients": [
        -2,
        0,
        1
      ],
      "lower": 0,
      "upper": 2,
      "start": 1,
      "tolerance": 1e-10,
      "max_iterations": 80
    }
  },
  {
    "id": "simpson",
    "title": "Y-02 複合Simpson積分",
    "doc": "numerical_analysis/simpson.md",
    "sample": {
      "coefficients": [
        0,
        0,
        0,
        1
      ],
      "lower": 0,
      "upper": 1,
      "subdivisions": 8
    }
  },
  {
    "id": "adaptive_simpson",
    "title": "Y-02 適応Simpson積分",
    "doc": "numerical_analysis/adaptive_simpson.md",
    "sample": {
      "function": "exp",
      "lower": 0,
      "upper": 1,
      "tolerance": 1e-08,
      "max_depth": 12
    }
  },
  {
    "id": "summation",
    "title": "Y-03 Kahan和・Neumaier補償和",
    "doc": "numerical_analysis/summation.md",
    "sample": {
      "values": [
        1e+30,
        1,
        -1e+30
      ]
    }
  },
  {
    "id": "log_sum_exp",
    "title": "Y-03 Log-Sum-Exp",
    "doc": "numerical_analysis/log_sum_exp.md",
    "sample": {
      "values": [
        1000,
        1000
      ]
    }
  },
  {
    "id": "tolerance",
    "title": "Y-03 絶対＋相対誤差比較",
    "doc": "numerical_analysis/tolerance.md",
    "sample": {
      "a": 1,
      "b": 1.0000001,
      "absolute": 1e-08,
      "relative": 1e-06
    }
  },
  {
    "id": "polynomial_roots",
    "title": "Y-04 多項式の実根列挙・Sturm分離",
    "doc": "numerical_analysis/polynomial_roots.md",
    "sample": {
      "coefficients": [
        0,
        -2,
        1,
        1
      ],
      "tolerance": 1e-07,
      "max_subdivisions": 400
    }
  },
  {
    "id": "tridiagonal",
    "title": "Y-05 pivot付き三重対角ソルバ",
    "doc": "numerical_analysis/tridiagonal.md",
    "sample": {
      "lower": [
        1,
        1
      ],
      "diagonal": [
        0,
        2,
        2
      ],
      "upper": [
        1,
        1
      ],
      "rhs": [
        2,
        8,
        8
      ]
    }
  }
];if(typeof module!=='undefined')module.exports=modes;else root.NumericalModes=modes;})(typeof window!=='undefined'?window:globalThis);
