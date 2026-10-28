(function(root){const modes=[
  {
    "id": "rectangle_area",
    "title": "H-01 長方形の和集合面積",
    "doc": "computational_geometry/rectangle_area.md",
    "sample": {
      "rectangles": [
        [
          0,
          0,
          3,
          2
        ],
        [
          2,
          1,
          4,
          3
        ]
      ]
    }
  },
  {
    "id": "rectangle_perimeter",
    "title": "H-02 長方形の和集合周長",
    "doc": "computational_geometry/rectangle_perimeter.md",
    "sample": {
      "rectangles": [
        [
          0,
          0,
          3,
          2
        ],
        [
          2,
          1,
          4,
          3
        ]
      ]
    }
  },
  {
    "id": "circle_tangents",
    "title": "H-03 円の接線",
    "doc": "computational_geometry/circle_tangents.md",
    "sample": {
      "circles": [
        [
          0,
          0,
          1
        ],
        [
          4,
          0,
          1
        ]
      ]
    }
  },
  {
    "id": "triangle_centers",
    "title": "H-04 三角形の外接円・内接円・垂心",
    "doc": "computational_geometry/triangle_centers.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          4,
          0
        ],
        [
          0,
          3
        ]
      ]
    }
  },
  {
    "id": "polygon_centroid",
    "title": "H-05 多角形の重心",
    "doc": "computational_geometry/polygon_centroid.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          4,
          0
        ],
        [
          4,
          2
        ],
        [
          0,
          2
        ]
      ]
    }
  },
  {
    "id": "circle_polygon_area",
    "title": "H-06 円と多角形の共通部分面積",
    "doc": "computational_geometry/circle_polygon_area.md",
    "sample": {
      "circle": [
        0,
        0,
        1
      ],
      "points": [
        [
          -2,
          -2
        ],
        [
          2,
          -2
        ],
        [
          2,
          2
        ],
        [
          -2,
          2
        ]
      ]
    }
  },
  {
    "id": "circle_intersection_area",
    "title": "H-07 2円の共通部分面積",
    "doc": "computational_geometry/circle_intersection_area.md",
    "sample": {
      "circles": [
        [
          0,
          0,
          1
        ],
        [
          0,
          0,
          1
        ]
      ]
    }
  },
  {
    "id": "circle_union_area",
    "title": "H-08 円の和集合面積",
    "doc": "computational_geometry/circle_union_area.md",
    "sample": {
      "circles": [
        [
          0,
          0,
          1
        ],
        [
          4,
          0,
          1
        ]
      ]
    }
  },
  {
    "id": "planar_embedding",
    "title": "H-09 平面埋め込みの面列挙・双対",
    "doc": "computational_geometry/planar_embedding.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          6,
          0
        ],
        [
          6,
          6
        ],
        [
          0,
          6
        ],
        [
          2,
          2
        ],
        [
          4,
          2
        ],
        [
          4,
          4
        ],
        [
          2,
          4
        ]
      ],
      "edges": [
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
        ],
        [
          3,
          0
        ],
        [
          4,
          5
        ],
        [
          5,
          6
        ],
        [
          6,
          7
        ],
        [
          7,
          4
        ]
      ]
    }
  },
  {
    "id": "rational_lattice",
    "title": "H-10 有理凸領域の格子点・整数凸包",
    "doc": "computational_geometry/rational_lattice.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          3,
          0
        ],
        [
          3,
          2
        ],
        [
          0,
          2
        ]
      ]
    }
  },
  {
    "id": "origin_rectangle_union",
    "title": "H-11 原点共有矩形の増分和集合",
    "doc": "computational_geometry/origin_rectangle_union.md",
    "sample": {
      "rectangles": [
        [
          2,
          3
        ],
        [
          4,
          1
        ],
        [
          3,
          2
        ]
      ]
    }
  },
  {
    "id": "segment_arrangement",
    "title": "H-12 線分sweep・交点列挙・Arrangement",
    "doc": "computational_geometry/segment_arrangement.md",
    "sample": {
      "segments": [
        [
          [
            0,
            0
          ],
          [
            4,
            4
          ]
        ],
        [
          [
            0,
            4
          ],
          [
            4,
            0
          ]
        ],
        [
          [
            0,
            2
          ],
          [
            4,
            2
          ]
        ]
      ]
    }
  },
  {
    "id": "convex_polygon_intersection",
    "title": "H-13 凸多角形の共通部分",
    "doc": "computational_geometry/convex_polygon_intersection.md",
    "sample": {
      "a": [
        [
          0,
          0
        ],
        [
          3,
          0
        ],
        [
          3,
          3
        ],
        [
          0,
          3
        ]
      ],
      "b": [
        [
          2,
          -1
        ],
        [
          4,
          -1
        ],
        [
          4,
          2
        ],
        [
          2,
          2
        ]
      ]
    }
  },
  {
    "id": "robust_predicates",
    "title": "H-14 適応精度・厳密幾何判定",
    "doc": "computational_geometry/robust_predicates.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          4,
          0
        ],
        [
          0,
          3
        ],
        [
          1,
          1
        ]
      ]
    }
  },
  {
    "id": "triangle_point_count",
    "title": "H-15 三角形内の点数クエリ",
    "doc": "computational_geometry/triangle_point_count.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          1,
          1
        ],
        [
          2,
          0
        ],
        [
          3,
          3
        ],
        [
          1,
          0
        ]
      ],
      "triangle": [
        [
          0,
          0
        ],
        [
          3,
          0
        ],
        [
          0,
          3
        ]
      ]
    }
  },
  {
    "id": "convex_farthest_neighbors",
    "title": "H-16 凸多角形の全頂点の最遠点",
    "doc": "computational_geometry/convex_farthest_neighbors.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          4,
          0
        ],
        [
          3,
          2
        ],
        [
          0,
          3
        ]
      ]
    }
  },
  {
    "id": "polygon_union_area",
    "title": "H-17 一般・凸多角形の和集合面積",
    "doc": "computational_geometry/polygon_union_area.md",
    "sample": {
      "polygons": [
        [
          [
            0,
            0
          ],
          [
            3,
            0
          ],
          [
            3,
            2
          ],
          [
            0,
            2
          ]
        ],
        [
          [
            2,
            1
          ],
          [
            4,
            1
          ],
          [
            4,
            3
          ],
          [
            2,
            3
          ]
        ]
      ]
    }
  },
  {
    "id": "delaunay",
    "title": "H-20 Delaunay三角形分割",
    "doc": "computational_geometry/delaunay.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          4,
          0
        ],
        [
          4,
          4
        ],
        [
          0,
          4
        ],
        [
          1,
          1
        ]
      ]
    }
  },
  {
    "id": "voronoi",
    "title": "H-21 Voronoi図",
    "doc": "computational_geometry/voronoi.md",
    "sample": {
      "points": [
        [
          1,
          1
        ],
        [
          3,
          1
        ],
        [
          2,
          3
        ]
      ],
      "box": [
        [
          0,
          0
        ],
        [
          4,
          4
        ]
      ]
    }
  },
  {
    "id": "dynamic_convex_hull",
    "title": "H-22 動的凸包",
    "doc": "computational_geometry/dynamic_convex_hull.md",
    "sample": {
      "operations": [
        [
          "insert",
          [
            0,
            0
          ]
        ],
        [
          "insert",
          [
            4,
            0
          ]
        ],
        [
          "insert",
          [
            0,
            3
          ]
        ],
        [
          "insert",
          [
            1,
            1
          ]
        ],
        [
          "erase",
          [
            1,
            1
          ]
        ]
      ],
      "direction": [
        1,
        1
      ]
    }
  },
  {
    "id": "polygon_triangulation",
    "title": "H-23 多角形の三角形分割",
    "doc": "computational_geometry/polygon_triangulation.md",
    "sample": {
      "points": [
        [
          0,
          0
        ],
        [
          4,
          0
        ],
        [
          4,
          4
        ],
        [
          2,
          2
        ],
        [
          0,
          4
        ]
      ]
    }
  },
  {
    "id": "geometry3d",
    "title": "H-24 三次元幾何",
    "doc": "computational_geometry/geometry3d.md",
    "sample": {
      "points": [
        [
          0,
          0,
          0
        ],
        [
          0,
          0,
          1
        ],
        [
          0,
          1,
          0
        ],
        [
          0,
          1,
          1
        ],
        [
          1,
          0,
          0
        ],
        [
          1,
          0,
          1
        ],
        [
          1,
          1,
          0
        ],
        [
          1,
          1,
          1
        ]
      ]
    }
  }
];if(typeof module!=="undefined")module.exports=modes;else root.GeometryExtensionModes=modes;})(typeof window!=="undefined"?window:globalThis);
