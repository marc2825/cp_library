(function(root){const modes=[
  {
    "id": "subsequence",
    "title": "S-01 部分列オートマトン・異なる部分列数",
    "doc": "string_algorithm/subsequence_automaton.md",
    "sample": {
      "text": "ababa",
      "pattern": "baa",
      "modulus": "1000000007"
    }
  },
  {
    "id": "lz77",
    "title": "S-02 Lempel–Ziv（LZ77）分解",
    "doc": "string_algorithm/lempel_ziv.md",
    "sample": {
      "text": "aaaaabaaaaab"
    }
  },
  {
    "id": "bwt",
    "title": "S-03 BWT・逆BWT",
    "doc": "string_algorithm/burrows_wheeler.md",
    "sample": {
      "text": "banana"
    }
  },
  {
    "id": "fm_index",
    "title": "S-04 FM-index",
    "doc": "string_algorithm/fm_index.md",
    "sample": {
      "text": "banana",
      "pattern": "ana",
      "sample_rate": 3
    }
  },
  {
    "id": "semi_local",
    "title": "S-05 Prefix-Substring / Semi-local LCS",
    "doc": "string_algorithm/semi_local_lcs.md",
    "sample": {
      "a": "abac",
      "b": "baca",
      "prefix": 3,
      "left": 1,
      "right": 4
    }
  },
  {
    "id": "borders",
    "title": "S-06 Border Tree・全周期列挙",
    "doc": "string_algorithm/border_tree.md",
    "sample": {
      "text": "ababab"
    }
  },
  {
    "id": "sam_utilities",
    "title": "S-07 SAMのk番目・全出現位置",
    "doc": "string_algorithm/suffix_automaton_utilities.md",
    "sample": {
      "text": "banana",
      "pattern": "ana",
      "k": 2
    }
  },
  {
    "id": "deque_eertree",
    "title": "S-20 両端回文木",
    "doc": "string_algorithm/double_ended_eertree.md",
    "sample": {
      "operations": [
        [
          "push_back",
          "a"
        ],
        [
          "push_back",
          "b"
        ],
        [
          "push_front",
          "a"
        ],
        [
          "push_back",
          "a"
        ],
        [
          "pop_front"
        ],
        [
          "push_front",
          "b"
        ]
      ]
    }
  },
  {
    "id": "online_z",
    "title": "S-21 Online Z",
    "doc": "string_algorithm/online_z.md",
    "sample": {
      "text": "ababa",
      "append": "b",
      "position": 2
    }
  },
  {
    "id": "generalized_sam",
    "title": "S-22 Generalized Suffix Automaton",
    "doc": "string_algorithm/generalized_suffix_automaton.md",
    "sample": {
      "documents": [
        "banana",
        "band",
        "ana"
      ],
      "pattern": "ana"
    }
  },
  {
    "id": "period_queries",
    "title": "S-23 区間の周期・反復回数",
    "doc": "string_algorithm/substring_period.md",
    "sample": {
      "text": "zzabababx",
      "left": 2,
      "right": 8,
      "period": 4
    }
  }
];if(typeof module!=="undefined")module.exports=modes;else root.StringExtensionModes=modes;})(typeof window!=="undefined"?window:globalThis);
