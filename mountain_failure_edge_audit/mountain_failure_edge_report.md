# Mountain failure-edge audit

## Summary

| Variant | Steps | SimT | Terminated | Truncated | Success | Collision | Min clearance | Dist goal | Score | Terrain | Min dist XY | First corr step | First corr target | Last pos | Last target |
|---|---:|---:|---|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| v1_locked | 3000 | 60.0 | False | True | False | False | 0.013782071080088255 | 59.00952764596378 | 0.01 | 1 | 21.419650615979247 | 251 | `[-39.0216, 44.4997, 29.6458]` | `[-88.4076, 75.7629, 14.4302]` | `[-39.0216, 44.4997, 29.6458]` |
| terrain3_override | 1125 | 22.5 | False | True | False | False | 0.703433298637303 | 24.04169967340434 | 0.01 | 3 | 17.468346278933897 | 251 | `[-39.0216, 44.4997, 29.6458]` | `[-56.9216, 54.5251, 13.8866]` | `[-39.0216, 44.4997, 29.6458]` |
| cruise_shim | 3000 | 60.0 | False | True | False | False | 0.9679740683223629 | 22.995092760059375 | 0.01 | 1 | 13.933875927267545 | 251 | `[-39.0216, 44.4997, 29.6458]` | `[-57.3313, 53.2554, 15.3052]` | `[-39.0216, 44.4997, 29.6458]` |
| xy_push | 1150 | 23.0 | False | True | False | False | 1.0 | 23.209852381142802 | 0.01 | 1 | 19.513609104301267 | 251 | `[-39.0216, 44.4997, 29.6458]` | `[-56.5675, 54.6534, 15.0628]` | `[-39.0216, 44.4997, 29.6458]` |

## Tail step info

### v1_locked

```json
[
  {
    "step": 2726,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 60.60583496816221,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.4551999866962433,
      -0.609499990940094,
      0.6491000056266785,
      0.8730000257492065,
      -0.006000000052154064
    ],
    "last_event": {
      "step": 2725,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 60.1609,
      "pos": [
        -88.7623,
        78.3404,
        14.6388
      ],
      "vel": [
        0.1274,
        0.2028,
        -0.6404
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        49.7407,
        -33.8407,
        13.0071
      ],
      "dist_xy": 60.1609,
      "agl": 7.6234,
      "control_output": [
        1.1922,
        -1.5962,
        1.7,
        2.619,
        -0.006
      ]
    }
  },
  {
    "step": 2751,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 59.43715097483473,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.47029998898506165,
      -0.64410001039505,
      0.6032000184059143,
      0.9394000172615051,
      -0.3635999858379364
    ],
    "last_event": {
      "step": 2750,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 59.169,
      "pos": [
        -88.1281,
        77.5077,
        15.4809
      ],
      "vel": [
        0.0096,
        0.0256,
        -0.4681
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        49.1065,
        -33.008,
        12.1649
      ],
      "dist_xy": 59.169,
      "agl": 7.6623,
      "control_output": [
        1.3255,
        -1.8152,
        1.7,
        2.8181,
        -0.3636
      ]
    }
  },
  {
    "step": 2776,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 58.277874018573954,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.2628999948501587,
      -0.7490000128746033,
      0.6082000136375427,
      0.9316999912261963,
      -0.15850000083446503
    ],
    "last_event": {
      "step": 2775,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 58.1759,
      "pos": [
        -87.5654,
        76.561,
        16.3254
      ],
      "vel": [
        0.1081,
        -0.0293,
        -0.5282
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        48.5439,
        -32.0613,
        11.3204
      ],
      "dist_xy": 58.1759,
      "agl": 7.4968,
      "control_output": [
        0.7349,
        -2.0936,
        1.7,
        2.7952,
        -0.1585
      ]
    }
  },
  {
    "step": 2801,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 57.26707954301404,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.2687999904155731,
      -0.7936999797821045,
      0.5457000136375427,
      0.8331999778747559,
      -0.5019999742507935
    ],
    "last_event": {
      "step": 2800,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 57.3286,
      "pos": [
        -87.2276,
        75.528,
        17.1556
      ],
      "vel": [
        -0.018,
        -0.0344,
        -0.5953
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        48.206,
        -31.0283,
        10.4902
      ],
      "dist_xy": 57.3286,
      "agl": 6.7778,
      "control_output": [
        0.6719,
        -1.984,
        1.364,
        2.4996,
        -0.502
      ]
    }
  },
  {
    "step": 2826,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 56.33095695786306,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.37610000371932983,
      -0.8141999840736389,
      0.4422999918460846,
      0.557699978351593,
      -0.1128000020980835
    ],
    "last_event": {
      "step": 2825,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 56.5078,
      "pos": [
        -86.8604,
        74.5759,
        17.7345
      ],
      "vel": [
        -0.0902,
        -0.2264,
        -0.5989
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        47.8388,
        -30.0762,
        9.9114
      ],
      "dist_xy": 56.5078,
      "agl": 5.8004,
      "control_output": [
        0.6292,
        -1.3623,
        0.74,
        1.6732,
        -0.1128
      ]
    }
  },
  {
    "step": 2851,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 55.63593671384905,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.5321999788284302,
      -0.7932999730110168,
      0.2957000136375427,
      0.3472000062465668,
      0.15000000596046448
    ],
    "last_event": {
      "step": 2850,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 55.8491,
      "pos": [
        -86.4958,
        73.9159,
        17.9773
      ],
      "vel": [
        -0.055,
        -0.1082,
        -0.5444
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        47.4743,
        -29.4162,
        9.6686
      ],
      "dist_xy": 55.8491,
      "agl": 4.9297,
      "control_output": [
        0.5544,
        -0.8262,
        0.308,
        1.0415,
        0.15
      ]
    }
  },
  {
    "step": 2876,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 55.22934437320159,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      0.4302000105381012,
      -0.3188999891281128,
      -0.8445000052452087,
      0.23839999735355377,
      -0.2459000051021576
    ],
    "last_event": {
      "step": 2875,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 55.4138,
      "pos": [
        -86.2143,
        73.5433,
        17.9132
      ],
      "vel": [
        -0.0796,
        -0.0994,
        -0.431
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        47.1928,
        -29.0436,
        9.7326
      ],
      "dist_xy": 55.4138,
      "agl": 4.3604,
      "control_output": [
        0.3077,
        -0.228,
        -0.604,
        0.7152,
        -0.2459
      ]
    }
  },
  {
    "step": 2901,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 55.23322945864614,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.0494999997317791,
      0.42149999737739563,
      -0.9054999947547913,
      0.5522000193595886,
      0.013799999840557575
    ],
    "last_event": {
      "step": 2900,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 55.2895,
      "pos": [
        -86.071,
        73.539,
        17.3786
      ],
      "vel": [
        -0.1628,
        -0.2168,
        -0.3984
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        47.0494,
        -29.0392,
        10.2672
      ],
      "dist_xy": 55.2895,
      "agl": 3.8,
      "control_output": [
        -0.082,
        0.6983,
        -1.5,
        1.6566,
        0.0138
      ]
    }
  },
  {
    "step": 2926,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 55.74985316046975,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.3840999901294708,
      0.5920000076293945,
      -0.7085000276565552,
      0.7056999802589417,
      -0.00559999980032444
    ],
    "last_event": {
      "step": 2925,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 55.6357,
      "pos": [
        -86.2065,
        73.9771,
        16.6318
      ],
      "vel": [
        0.002,
        -0.2016,
        -0.4904
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        47.185,
        -29.4774,
        11.014
      ],
      "dist_xy": 55.6357,
      "agl": 3.6026,
      "control_output": [
        -0.8132,
        1.2534,
        -1.5,
        2.1171,
        -0.0056
      ]
    }
  },
  {
    "step": 2951,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 56.674811069810616,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.5737000107765198,
      0.5529000163078308,
      -0.6043000221252441,
      0.8274000287055969,
      -0.3427000045776367
    ],
    "last_event": {
      "step": 2950,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 56.3951,
      "pos": [
        -86.701,
        74.6175,
        15.887
      ],
      "vel": [
        0.0653,
        -0.0913,
        -0.6831
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        47.6795,
        -30.1178,
        11.7588
      ],
      "dist_xy": 56.3951,
      "agl": 3.9657,
      "control_output": [
        -1.424,
        1.3724,
        -1.5,
        2.4821,
        -0.3427
      ]
    }
  },
  {
    "step": 2976,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 57.84147962023092,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.7279000282287598,
      0.38609999418258667,
      -0.5666999816894531,
      0.8823000192642212,
      -0.21709999442100525
    ],
    "last_event": {
      "step": 2975,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 57.4115,
      "pos": [
        -87.4636,
        75.3128,
        15.1443
      ],
      "vel": [
        0.1679,
        -0.0297,
        -0.6577
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        48.4421,
        -30.8131,
        12.5015
      ],
      "dist_xy": 57.4115,
      "agl": 4.5152,
      "control_output": [
        -1.9267,
        1.0219,
        -1.5,
        2.647,
        -0.2171
      ]
    }
  },
  {
    "step": 3000,
    "terminated": false,
    "truncated": true,
    "info": {
      "distance_to_goal": 59.00952764596378,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.013782071080088255,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.7677000164985657,
      0.2980000078678131,
      -0.5673999786376953,
      0.8812999725341797,
      -0.4239000082015991
    ],
    "last_event": {
      "step": 2999,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 58.4497,
      "pos": [
        -88.4076,
        75.7629,
        14.4302
      ],
      "vel": [
        0.0319,
        -0.0012,
        -0.721
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        49.3861,
        -31.2632,
        13.2157
      ],
      "dist_xy": 58.4497,
      "agl": 4.4537,
      "control_output": [
        -2.0296,
        0.7878,
        -1.5,
        2.6439,
        -0.4239
      ]
    }
  }
]
```

### terrain3_override

```json
[
  {
    "step": 851,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.28570878674279,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.6432999968528748,
      0.5144000053405762,
      -0.5669999718666077,
      0.8817999958992004,
      -0.11949999630451202
    ],
    "last_event": {
      "step": 850,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.2399,
      "pos": [
        -56.03,
        53.4934,
        16.9574
      ],
      "vel": [
        -0.0701,
        -0.1193,
        -0.2862
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.0085,
        -8.9937,
        10.6884
      ],
      "dist_xy": 19.2399,
      "agl": 4.5879,
      "control_output": [
        -1.7018,
        1.3608,
        -1.5,
        2.6454,
        -0.1195
      ]
    }
  },
  {
    "step": 876,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.52161384804153,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.8294000029563904,
      -0.44749999046325684,
      0.3343999981880188,
      0.5482000112533569,
      -0.7357000112533569
    ],
    "last_event": {
      "step": 875,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.3277,
      "pos": [
        -56.9114,
        54.152,
        16.3504
      ],
      "vel": [
        0.0368,
        0.0641,
        -0.5271
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.8898,
        -9.6523,
        11.2955
      ],
      "dist_xy": 20.3277,
      "agl": 4.7287,
      "control_output": [
        -1.8713,
        0.8612,
        -1.116,
        2.3428,
        -0.7357
      ]
    }
  },
  {
    "step": 901,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.51240446730651,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.8334000110626221,
      -0.4399999976158142,
      0.3343999981880188,
      0.5482000112533569,
      -0.10320000350475311
    ],
    "last_event": {
      "step": 900,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.6244,
      "pos": [
        -57.2606,
        54.128,
        16.7081
      ],
      "vel": [
        -0.5712,
        -0.0782,
        -2.631
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        18.2391,
        -9.6283,
        10.9378
      ],
      "dist_xy": 20.6244,
      "agl": 5.225,
      "control_output": [
        -1.1057,
        0.7815,
        0.084,
        1.3566,
        -0.1032
      ]
    }
  },
  {
    "step": 926,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.54422687777723,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.1454000025987625,
      0.3903000056743622,
      0.9090999960899353,
      0.15399999916553497,
      -0.6571000218391418
    ],
    "last_event": {
      "step": 925,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.6937,
      "pos": [
        -56.5945,
        53.3897,
        16.984
      ],
      "vel": [
        0.7167,
        0.3998,
        -2.9238
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.573,
        -8.89,
        10.6619
      ],
      "dist_xy": 19.6937,
      "agl": 4.8026,
      "control_output": [
        -0.0672,
        0.1803,
        0.42,
        0.462,
        -0.6571
      ]
    }
  },
  {
    "step": 951,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.715255907955125,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.7411999702453613,
      -0.27489998936653137,
      -0.6123999953269958,
      0.4246000051498413,
      0.12020000070333481
    ],
    "last_event": {
      "step": 950,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.7305,
      "pos": [
        -56.5559,
        53.5464,
        16.8356
      ],
      "vel": [
        -0.2731,
        -0.2522,
        -3.0189
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.5343,
        -9.0466,
        10.8102
      ],
      "dist_xy": 19.7305,
      "agl": 4.7242,
      "control_output": [
        0.9442,
        -0.3502,
        -0.78,
        1.2738,
        0.1202
      ]
    }
  },
  {
    "step": 976,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.785989315597806,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.6136999726295471,
      -0.43529999256134033,
      -0.6586999893188477,
      0.7591000199317932,
      0.8083000183105469
    ],
    "last_event": {
      "step": 975,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.4642,
      "pos": [
        -56.2874,
        53.4855,
        16.2409
      ],
      "vel": [
        -0.1693,
        0.1768,
        -1.2252
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.2659,
        -8.9858,
        11.405
      ],
      "dist_xy": 19.4642,
      "agl": 3.979,
      "control_output": [
        1.3975,
        -0.9913,
        -1.5,
        2.2773,
        0.8083
      ]
    }
  },
  {
    "step": 1001,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.493398396196543,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.6126000285148621,
      -0.12809999287128448,
      -0.7799999713897705,
      0.6410999894142151,
      -0.3682999908924103
    ],
    "last_event": {
      "step": 1000,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 18.5695,
      "pos": [
        -55.426,
        53.2014,
        15.4958
      ],
      "vel": [
        -0.0783,
        -0.2457,
        -0.7646
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        16.4045,
        -8.7017,
        12.1501
      ],
      "dist_xy": 18.5695,
      "agl": 2.8317,
      "control_output": [
        1.1781,
        -0.2464,
        -1.5,
        1.9232,
        -0.3683
      ]
    }
  },
  {
    "step": 1026,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.552537234601388,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.5067999958992004,
      0.48820000886917114,
      -0.7105000019073486,
      0.2759000062942505,
      -0.04349999874830246
    ],
    "last_event": {
      "step": 1025,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 18.1501,
      "pos": [
        -54.9928,
        53.1222,
        14.8839
      ],
      "vel": [
        0.0334,
        -0.0881,
        -0.5456
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        15.9712,
        -8.6225,
        12.7619
      ],
      "dist_xy": 18.1501,
      "agl": 2.084,
      "control_output": [
        0.4195,
        0.404,
        -0.588,
        0.8276,
        -0.0435
      ]
    }
  },
  {
    "step": 1051,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.75753937760314,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.3677000105381012,
      0.6625999808311462,
      -0.6524999737739563,
      0.47200000286102295,
      -0.021299999207258224
    ],
    "last_event": {
      "step": 1050,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 18.1732,
      "pos": [
        -54.8782,
        53.3785,
        14.6484
      ],
      "vel": [
        -0.0694,
        -0.3571,
        -0.3015
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        15.8566,
        -8.8788,
        12.9974
      ],
      "dist_xy": 18.1732,
      "agl": 1.9566,
      "control_output": [
        -0.5206,
        0.9383,
        -0.924,
        1.4161,
        -0.0213
      ]
    }
  },
  {
    "step": 1076,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.650811486953856,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.6926000118255615,
      0.42250001430511475,
      -0.5846999883651733,
      0.7182999849319458,
      -0.1867000013589859
    ],
    "last_event": {
      "step": 1075,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 18.7521,
      "pos": [
        -55.2771,
        53.8484,
        14.0045
      ],
      "vel": [
        0.1185,
        -0.1239,
        -0.5599
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        16.2555,
        -9.3487,
        13.6414
      ],
      "dist_xy": 18.7521,
      "agl": 1.6441,
      "control_output": [
        -1.4924,
        0.9104,
        -1.26,
        2.1549,
        -0.1867
      ]
    }
  },
  {
    "step": 1101,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.595270262113434,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.9483000040054321,
      0.3160000145435333,
      -0.027899999171495438,
      0.7164000272750854,
      -0.09179999679327011
    ],
    "last_event": {
      "step": 1100,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.6813,
      "pos": [
        -56.1065,
        54.2698,
        13.6251
      ],
      "vel": [
        0.0691,
        -0.0532,
        -0.5721
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.085,
        -9.7701,
        14.0207
      ],
      "dist_xy": 19.6813,
      "agl": 1.7187,
      "control_output": [
        -2.0381,
        0.6792,
        -0.06,
        2.1491,
        -0.0918
      ]
    }
  },
  {
    "step": 1125,
    "terminated": false,
    "truncated": true,
    "info": {
      "distance_to_goal": 24.04169967340434,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.703433298637303,
      "landing_stable_time": 0.0
    },
    "action": [
      0.8222000002861023,
      -0.46050000190734863,
      0.3343999981880188,
      0.5482000112533569,
      -0.5612999796867371
    ],
    "last_event": {
      "step": 1124,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.5163,
      "pos": [
        -56.9216,
        54.5251,
        13.8866
      ],
      "vel": [
        -0.9988,
        0.609,
        -1.9689
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.9,
        -10.0253,
        13.7592
      ],
      "dist_xy": 20.5163,
      "agl": 2.4759,
      "control_output": [
        -1.6961,
        0.6228,
        1.092,
        2.1112,
        -0.5613
      ]
    }
  }
]
```

### cruise_shim

```json
[
  {
    "step": 2726,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 24.666500792869627,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.3495999872684479,
      -0.08089999854564667,
      0.9333999752998352,
      0.5613999962806702,
      -0.05869999900460243
    ],
    "last_event": {
      "step": 2725,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 22.2977,
      "pos": [
        -58.8847,
        54.6309,
        15.2941
      ],
      "vel": [
        0.0604,
        0.1744,
        -0.3849
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        19.8632,
        -10.1311,
        12.3517
      ],
      "dist_xy": 22.2977,
      "agl": 4.8211,
      "control_output": [
        -0.5888,
        -0.1363,
        1.572,
        1.6842,
        -0.0587
      ]
    }
  },
  {
    "step": 2751,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 24.287710180974713,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      0.2946000099182129,
      -0.27959999442100525,
      0.9138000011444092,
      0.6201000213623047,
      -0.179299995303154
    ],
    "last_event": {
      "step": 2750,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 22.3922,
      "pos": [
        -59.0572,
        54.4988,
        16.1285
      ],
      "vel": [
        -0.066,
        0.2612,
        -0.6182
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        20.0357,
        -9.999,
        11.5173
      ],
      "dist_xy": 22.3922,
      "agl": 5.6789,
      "control_output": [
        0.5481,
        -0.5202,
        1.7,
        1.8604,
        -0.1793
      ]
    }
  },
  {
    "step": 2776,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.40067186517589,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      0.6606000065803528,
      -0.34860000014305115,
      0.6647999882698059,
      0.8522999882698059,
      0.014499999582767487
    ],
    "last_event": {
      "step": 2775,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 21.8764,
      "pos": [
        -58.6362,
        54.187,
        16.9726
      ],
      "vel": [
        -0.0946,
        0.2222,
        -0.7516
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        19.6146,
        -9.6873,
        10.6732
      ],
      "dist_xy": 21.8764,
      "agl": 6.1657,
      "control_output": [
        1.6892,
        -0.8915,
        1.7,
        2.557,
        0.0145
      ]
    }
  },
  {
    "step": 2801,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.06625030011944,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      0.715499997138977,
      -0.3531000018119812,
      0.6028000116348267,
      0.9399999976158142,
      0.16140000522136688
    ],
    "last_event": {
      "step": 2800,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.8214,
      "pos": [
        -57.6928,
        53.7148,
        17.8137
      ],
      "vel": [
        0.0007,
        -0.0232,
        -0.5933
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        18.6712,
        -9.2151,
        9.8322
      ],
      "dist_xy": 20.8214,
      "agl": 6.2245,
      "control_output": [
        2.0176,
        -0.9958,
        1.7,
        2.82,
        0.1614
      ]
    }
  },
  {
    "step": 2826,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 20.713013879236748,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      0.7404000163078308,
      -0.46239998936653137,
      0.4878999888896942,
      0.9319000244140625,
      -0.12189999967813492
    ],
    "last_event": {
      "step": 2825,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.7171,
      "pos": [
        -56.6974,
        53.2363,
        18.6455
      ],
      "vel": [
        0.0444,
        0.1448,
        -0.6086
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.6758,
        -8.7366,
        9.0003
      ],
      "dist_xy": 19.7171,
      "agl": 6.424,
      "control_output": [
        2.0698,
        -1.2926,
        1.364,
        2.7956,
        -0.1219
      ]
    }
  },
  {
    "step": 2851,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 19.470807888660776,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      0.8296999931335449,
      -0.5526999831199646,
      0.0786999985575676,
      0.6944000124931335,
      -0.03999999910593033
    ],
    "last_event": {
      "step": 2850,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 18.5396,
      "pos": [
        -55.6965,
        52.603,
        19.0774
      ],
      "vel": [
        -0.039,
        -0.1324,
        -0.2828
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        16.675,
        -8.1033,
        8.5684
      ],
      "dist_xy": 18.5396,
      "agl": 6.1615,
      "control_output": [
        1.7284,
        -1.1515,
        0.164,
        2.0833,
        -0.04
      ]
    }
  },
  {
    "step": 2876,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 18.780139179061553,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      0.5773000121116638,
      -0.33629998564720154,
      -0.7440999746322632,
      0.4641000032424927,
      -0.0851999968290329
    ],
    "last_event": {
      "step": 2875,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 17.6368,
      "pos": [
        -54.9415,
        52.09,
        18.9164
      ],
      "vel": [
        -0.139,
        -0.256,
        -0.125
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        15.9199,
        -7.5903,
        8.7294
      ],
      "dist_xy": 17.6368,
      "agl": 5.5633,
      "control_output": [
        0.8037,
        -0.4682,
        -1.036,
        1.3923,
        -0.0852
      ]
    }
  },
  {
    "step": 2901,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 18.888755031404017,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.12610000371932983,
      0.031700000166893005,
      -0.9915000200271606,
      0.5042999982833862,
      -0.3003999888896942
    ],
    "last_event": {
      "step": 2900,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 17.3324,
      "pos": [
        -54.6702,
        51.9518,
        18.2393
      ],
      "vel": [
        -0.0307,
        -0.1965,
        -0.2379
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        15.6486,
        -7.4521,
        9.4065
      ],
      "dist_xy": 17.3324,
      "agl": 4.2883,
      "control_output": [
        -0.1907,
        0.0479,
        -1.5,
        1.5128,
        -0.3004
      ]
    }
  },
  {
    "step": 2926,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 19.495903376242993,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.5546000003814697,
      0.2856000065803528,
      -0.781499981880188,
      0.6398000121116638,
      -0.05869999900460243
    ],
    "last_event": {
      "step": 2925,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 17.5587,
      "pos": [
        -54.8726,
        52.0529,
        17.4947
      ],
      "vel": [
        -0.0516,
        -0.1909,
        -0.3138
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        15.8511,
        -7.5532,
        10.1512
      ],
      "dist_xy": 17.5587,
      "agl": 3.988,
      "control_output": [
        -1.0645,
        0.5482,
        -1.5,
        1.9193,
        -0.0587
      ]
    }
  },
  {
    "step": 2951,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 20.495564969500073,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.7060999870300293,
      0.33180001378059387,
      -0.6255999803543091,
      0.7993000149726868,
      -0.2874000072479248
    ],
    "last_event": {
      "step": 2950,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 18.2489,
      "pos": [
        -55.4768,
        52.3893,
        16.7496
      ],
      "vel": [
        0.0056,
        -0.1419,
        -0.2868
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        16.4553,
        -7.8895,
        10.8962
      ],
      "dist_xy": 18.2489,
      "agl": 3.6746,
      "control_output": [
        -1.693,
        0.7957,
        -1.5,
        2.3978,
        -0.2874
      ]
    }
  },
  {
    "step": 2976,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 21.7271622205206,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.7501000165939331,
      0.34360000491142273,
      -0.5651000142097473,
      0.8848000168800354,
      -0.006000000052154064
    ],
    "last_event": {
      "step": 2975,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.2318,
      "pos": [
        -56.3657,
        52.8089,
        16.0058
      ],
      "vel": [
        0.0291,
        -0.071,
        -0.5327
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.3442,
        -8.3092,
        11.6401
      ],
      "dist_xy": 19.2318,
      "agl": 3.4034,
      "control_output": [
        -1.991,
        0.912,
        -1.5,
        2.6544,
        -0.006
      ]
    }
  },
  {
    "step": 3000,
    "terminated": false,
    "truncated": true,
    "info": {
      "distance_to_goal": 22.995092760059375,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 0.9679740683223629,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.7965999841690063,
      0.3928999900817871,
      -0.4595000147819519,
      0.8443999886512756,
      -0.13760000467300415
    ],
    "last_event": {
      "step": 2999,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.2955,
      "pos": [
        -57.3313,
        53.2554,
        15.3052
      ],
      "vel": [
        -0.036,
        -0.0455,
        -0.4648
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        18.3097,
        -8.7556,
        12.3406
      ],
      "dist_xy": 20.2955,
      "agl": 3.3439,
      "control_output": [
        -2.0179,
        0.9952,
        -1.164,
        2.5333,
        -0.1376
      ]
    }
  }
]
```

### xy_push

```json
[
  {
    "step": 876,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.32321009452926,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.2709999978542328,
      0.3249000012874603,
      0.20350000262260437,
      0.3698999881744385,
      0.6136000156402588
    ],
    "last_event": {
      "step": 875,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.2302,
      "pos": [
        -57.2155,
        53.3451,
        14.5436
      ],
      "vel": [
        0.0776,
        0.1329,
        0.5177
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        18.1939,
        -8.8454,
        13.1022
      ],
      "dist_xy": 20.2302,
      "agl": 2.6076,
      "control_output": [
        -0.2698,
        1.0524,
        0.2258,
        1.1097,
        0.6136
      ]
    }
  },
  {
    "step": 901,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.02402293078606,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.4449000060558319,
      0.8616999983787537,
      -0.24400000274181366,
      0.41280001401901245,
      0.6517999768257141
    ],
    "last_event": {
      "step": 900,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.9628,
      "pos": [
        -56.7407,
        53.6944,
        14.7941
      ],
      "vel": [
        -0.7317,
        -0.1781,
        0.8448
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.7192,
        -9.1947,
        12.8518
      ],
      "dist_xy": 19.9628,
      "agl": 2.8449,
      "control_output": [
        -0.5509,
        1.067,
        -0.3022,
        1.2383,
        0.6518
      ]
    }
  },
  {
    "step": 926,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.885946618726262,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.46970000863075256,
      -0.06830000132322311,
      -0.9560999870300293,
      0.3898000121116638,
      0.7742999792098999
    ],
    "last_event": {
      "step": 925,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.5077,
      "pos": [
        -57.1072,
        54.1681,
        14.1643
      ],
      "vel": [
        0.3228,
        -0.2275,
        0.6627
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        18.0856,
        -9.6684,
        13.4815
      ],
      "dist_xy": 20.5077,
      "agl": 2.6367,
      "control_output": [
        0.1548,
        0.3059,
        -1.1182,
        1.1695,
        0.7743
      ]
    }
  },
  {
    "step": 951,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 24.09639406405389,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.8916000127792358,
      0.013700000010430813,
      0.21220000088214874,
      0.2199999988079071,
      0.609499990940094
    ],
    "last_event": {
      "step": 950,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.6248,
      "pos": [
        -57.3835,
        53.8923,
        13.8352
      ],
      "vel": [
        0.0455,
        0.2598,
        1.1315
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        18.362,
        -9.3926,
        13.8106
      ],
      "dist_xy": 20.6248,
      "agl": 2.2576,
      "control_output": [
        0.3444,
        0.1533,
        0.0818,
        0.3857,
        0.6095
      ]
    }
  },
  {
    "step": 976,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.71534780267877,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.5076000094413757,
      -0.4700999855995178,
      0.8580999970436096,
      0.49790000915527344,
      -0.7548999786376953
    ],
    "last_event": {
      "step": 975,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.3389,
      "pos": [
        -57.0131,
        53.9853,
        14.104
      ],
      "vel": [
        0.0722,
        -0.1738,
        1.0876
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.9915,
        -9.4856,
        13.5418
      ],
      "dist_xy": 20.3389,
      "agl": 2.4343,
      "control_output": [
        0.2976,
        -0.7069,
        1.2818,
        1.4938,
        -0.7549
      ]
    }
  },
  {
    "step": 1001,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.899423187136986,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.019099999219179153,
      -0.45019999146461487,
      0.8927000164985657,
      0.47929999232292175,
      0.7371000051498413
    ],
    "last_event": {
      "step": 1000,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.885,
      "pos": [
        -56.7211,
        53.563,
        14.8142
      ],
      "vel": [
        -0.2752,
        -0.0604,
        0.9953
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.6995,
        -9.0632,
        12.8316
      ],
      "dist_xy": 19.885,
      "agl": 2.7838,
      "control_output": [
        0.0275,
        -0.6475,
        1.2837,
        1.438,
        0.7371
      ]
    }
  },
  {
    "step": 1026,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.539246811804052,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.2257000058889389,
      -0.9664000272750854,
      0.12330000102519989,
      0.2264000028371811,
      -0.5728999972343445
    ],
    "last_event": {
      "step": 1025,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.7092,
      "pos": [
        -56.6805,
        53.2527,
        15.2071
      ],
      "vel": [
        0.2394,
        -0.0081,
        1.1805
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.659,
        -8.753,
        12.4387
      ],
      "dist_xy": 19.7092,
      "agl": 2.9873,
      "control_output": [
        0.1532,
        -0.6562,
        0.0837,
        0.6791,
        -0.5729
      ]
    }
  },
  {
    "step": 1051,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.52903266929466,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.19670000672340393,
      0.2084999978542328,
      -0.9580000042915344,
      0.38839998841285706,
      0.30000001192092896
    ],
    "last_event": {
      "step": 1050,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.5639,
      "pos": [
        -56.6429,
        52.999,
        15.0063
      ],
      "vel": [
        -0.2583,
        0.2947,
        0.6744
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.6213,
        -8.4993,
        12.6395
      ],
      "dist_xy": 19.5639,
      "agl": 2.6297,
      "control_output": [
        0.2292,
        0.2429,
        -1.1163,
        1.1651,
        0.3
      ]
    }
  },
  {
    "step": 1076,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 22.94170372609838,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.07479999959468842,
      0.7820000052452087,
      -0.6187999844551086,
      0.44600000977516174,
      -0.17910000681877136
    ],
    "last_event": {
      "step": 1075,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.564,
      "pos": [
        -56.508,
        53.2734,
        14.3753
      ],
      "vel": [
        -0.0862,
        -0.0705,
        0.2149
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.4864,
        -8.7736,
        13.2705
      ],
      "dist_xy": 19.564,
      "agl": 2.0919,
      "control_output": [
        -0.1001,
        1.0463,
        -0.828,
        1.3381,
        -0.1791
      ]
    }
  },
  {
    "step": 1101,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.38990947861797,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.6276999711990356,
      0.739300012588501,
      0.24400000274181366,
      0.5081999897956848,
      -0.31859999895095825
    ],
    "last_event": {
      "step": 1100,
      "phase": "pre_terrain",
      "goal_corr": "village",
      "init_dist": 41.9548,
      "raw_search_dist": 19.9538,
      "pos": [
        -56.6371,
        53.8724,
        14.2115
      ],
      "vel": [
        -0.038,
        -0.1765,
        0.0157
      ],
      "target": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.6156,
        -9.3726,
        13.4343
      ],
      "dist_xy": 19.9538,
      "agl": 2.3158,
      "control_output": [
        -0.957,
        1.1272,
        0.372,
        1.5247,
        -0.3186
      ]
    }
  },
  {
    "step": 1126,
    "terminated": false,
    "truncated": false,
    "info": {
      "distance_to_goal": 23.38959064798585,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      0.1965000033378601,
      0.21160000562667847,
      0.5134999752044678,
      0.4596000015735626,
      0.47209998965263367
    ],
    "last_event": {
      "step": 1125,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.4307,
      "pos": [
        -56.9241,
        54.3443,
        14.8568
      ],
      "vel": [
        0.3238,
        0.2808,
        0.7198
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.9025,
        -9.8445,
        12.7891
      ],
      "dist_xy": 20.4307,
      "agl": 3.347,
      "control_output": [
        -0.4959,
        1.0741,
        0.708,
        1.3787,
        0.4721
      ]
    }
  },
  {
    "step": 1150,
    "terminated": false,
    "truncated": true,
    "info": {
      "distance_to_goal": 23.209852381142802,
      "score": 0.01,
      "success": false,
      "collision": false,
      "t_to_goal": null,
      "min_clearance": 1.0,
      "landing_stable_time": 0.0
    },
    "action": [
      -0.006300000008195639,
      0.10239999741315842,
      -0.3564999997615814,
      0.41510000824928284,
      0.5612000226974487
    ],
    "last_event": {
      "step": 1149,
      "phase": "pre_terrain",
      "goal_corr": "mountain",
      "init_dist": 41.9548,
      "raw_search_dist": 20.2721,
      "pos": [
        -56.5675,
        54.6534,
        15.0628
      ],
      "vel": [
        -1.0088,
        0.0754,
        0.4565
      ],
      "target": [
        -39.0216,
        44.4997,
        29.6458
      ],
      "target_est": [
        -39.0216,
        44.4997,
        27.6458
      ],
      "search_vec": [
        17.5459,
        -10.1537,
        12.5831
      ],
      "dist_xy": 20.2721,
      "agl": 3.5691,
      "control_output": [
        -0.896,
        0.7421,
        -0.444,
        1.2453,
        0.5612
      ]
    }
  }
]
```
