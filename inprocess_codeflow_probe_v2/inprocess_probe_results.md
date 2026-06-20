# In-process code-flow probe v2

Agent class: `DroneFlightController`

## Runs

| Group | Steps | Terminated | Truncated | Success | Score | Events | First correction | First corr step | First init_dist | First raw_search | Last correction | Last target | Last target_est | Agent terrain_id | Agent min_dist_xy |
|---|---:|---|---|---|---:|---:|---|---:|---:|---:|---|---|---|---:|---:|
| type3_mountain | 3000 | False | True | False | 0.01 | 2759 | mountain | 251 | 41.9548 | 36.9353 | mountain | `[-39.0216, 44.4997, 29.6458]` | `[-39.0216, 44.4997, 27.6458]` | 1 | 21.419650615979247 |
| type4_village | 1765 | False | True | False | 0.01 | 1253 | mountain | 251 | 45.5988 | 41.6885 | village | `[-39.6732, 33.9357, 3.853]` | `[-39.6732, 33.9357, 1.5122]` | 5 | 1.252140433701417 |

## Correction event counts

- `type3_mountain`: `{'None': 10, 'mountain': 2749}`
- `type4_village`: `{'None': 13, 'mountain': 686, 'village': 554}`

## First 20 correction events per run

### type3_mountain

```json
[
  {
    "phase": "pre_terrain",
    "step": 251,
    "init_dist": 41.9548,
    "raw_search_dist": 36.9353,
    "goal_corr": "mountain",
    "pos": [
      -71.3853,
      62.2987,
      14.0206
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.3638,
      -17.799,
      13.6252
    ],
    "dist_xy": 36.9353,
    "agl": 4.8434,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 252,
    "init_dist": 41.9548,
    "raw_search_dist": 36.8912,
    "goal_corr": "mountain",
    "pos": [
      -71.3467,
      62.2775,
      14.0118
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.3251,
      -17.7777,
      13.634
    ],
    "dist_xy": 36.8912,
    "agl": 4.8436,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 253,
    "init_dist": 41.9548,
    "raw_search_dist": 36.8471,
    "goal_corr": "mountain",
    "pos": [
      -71.308,
      62.2562,
      14.0021
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.2865,
      -17.7564,
      13.6438
    ],
    "dist_xy": 36.8471,
    "agl": 4.8428,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 254,
    "init_dist": 41.9548,
    "raw_search_dist": 36.803,
    "goal_corr": "mountain",
    "pos": [
      -71.2694,
      62.2348,
      13.9913
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.2479,
      -17.7351,
      13.6545
    ],
    "dist_xy": 36.803,
    "agl": 4.8411,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 255,
    "init_dist": 41.9548,
    "raw_search_dist": 36.7589,
    "goal_corr": "mountain",
    "pos": [
      -71.2308,
      62.2135,
      13.9797
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.2093,
      -17.7138,
      13.6662
    ],
    "dist_xy": 36.7589,
    "agl": 4.8384,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 256,
    "init_dist": 41.9548,
    "raw_search_dist": 36.7148,
    "goal_corr": "mountain",
    "pos": [
      -71.1922,
      62.1922,
      13.9678
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.1706,
      -17.6925,
      13.6781
    ],
    "dist_xy": 36.7148,
    "agl": 4.8356,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 257,
    "init_dist": 41.9548,
    "raw_search_dist": 36.6706,
    "goal_corr": "mountain",
    "pos": [
      -71.1535,
      62.1708,
      13.9561
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.132,
      -17.6711,
      13.6898
    ],
    "dist_xy": 36.6706,
    "agl": 4.8329,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 258,
    "init_dist": 41.9548,
    "raw_search_dist": 36.6263,
    "goal_corr": "mountain",
    "pos": [
      -71.1148,
      62.1494,
      13.9448
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.0932,
      -17.6497,
      13.701
    ],
    "dist_xy": 36.6263,
    "agl": 4.8307,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 259,
    "init_dist": 41.9548,
    "raw_search_dist": 36.582,
    "goal_corr": "mountain",
    "pos": [
      -71.076,
      62.128,
      13.9342
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.0544,
      -17.6283,
      13.7116
    ],
    "dist_xy": 36.582,
    "agl": 4.8292,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 260,
    "init_dist": 41.9548,
    "raw_search_dist": 36.5376,
    "goal_corr": "mountain",
    "pos": [
      -71.0371,
      62.1067,
      13.9244
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      32.0155,
      -17.6069,
      13.7214
    ],
    "dist_xy": 36.5376,
    "agl": 4.8322,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 261,
    "init_dist": 41.9548,
    "raw_search_dist": 36.4933,
    "goal_corr": "mountain",
    "pos": [
      -70.9982,
      62.0853,
      13.9155
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.9767,
      -17.5856,
      13.7304
    ],
    "dist_xy": 36.4933,
    "agl": 4.8484,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 262,
    "init_dist": 41.9548,
    "raw_search_dist": 36.4489,
    "goal_corr": "mountain",
    "pos": [
      -70.9593,
      62.064,
      13.9074
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.9377,
      -17.5643,
      13.7385
    ],
    "dist_xy": 36.4489,
    "agl": 4.8655,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 263,
    "init_dist": 41.9548,
    "raw_search_dist": 36.4046,
    "goal_corr": "mountain",
    "pos": [
      -70.9204,
      62.0428,
      13.9002
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.8988,
      -17.5431,
      13.7456
    ],
    "dist_xy": 36.4046,
    "agl": 4.8834,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 264,
    "init_dist": 41.9548,
    "raw_search_dist": 36.3603,
    "goal_corr": "mountain",
    "pos": [
      -70.8815,
      62.0215,
      13.894
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.86,
      -17.5218,
      13.7519
    ],
    "dist_xy": 36.3603,
    "agl": 4.9023,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 265,
    "init_dist": 41.9548,
    "raw_search_dist": 36.3161,
    "goal_corr": "mountain",
    "pos": [
      -70.8427,
      62.0004,
      13.8887
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.8211,
      -17.5007,
      13.7572
    ],
    "dist_xy": 36.3161,
    "agl": 4.9221,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 266,
    "init_dist": 41.9548,
    "raw_search_dist": 36.2719,
    "goal_corr": "mountain",
    "pos": [
      -70.8039,
      61.9793,
      13.8843
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.7823,
      -17.4795,
      13.7615
    ],
    "dist_xy": 36.2719,
    "agl": 4.9427,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 267,
    "init_dist": 41.9548,
    "raw_search_dist": 36.2278,
    "goal_corr": "mountain",
    "pos": [
      -70.7651,
      61.9582,
      13.8809
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.7435,
      -17.4585,
      13.765
    ],
    "dist_xy": 36.2278,
    "agl": 4.9643,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 268,
    "init_dist": 41.9548,
    "raw_search_dist": 36.1837,
    "goal_corr": "mountain",
    "pos": [
      -70.7263,
      61.9372,
      13.8784
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.7048,
      -17.4375,
      13.7675
    ],
    "dist_xy": 36.1837,
    "agl": 4.9868,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 269,
    "init_dist": 41.9548,
    "raw_search_dist": 36.1396,
    "goal_corr": "mountain",
    "pos": [
      -70.6875,
      61.9163,
      13.8769
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.666,
      -17.4165,
      13.769
    ],
    "dist_xy": 36.1396,
    "agl": 5.0103,
    "_group": "type3_mountain",
    "_seed": 604195
  },
  {
    "phase": "pre_terrain",
    "step": 270,
    "init_dist": 41.9548,
    "raw_search_dist": 36.0955,
    "goal_corr": "mountain",
    "pos": [
      -70.6487,
      61.8953,
      13.8763
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
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      31.6272,
      -17.3956,
      13.7696
    ],
    "dist_xy": 36.0955,
    "agl": 5.0347,
    "_group": "type3_mountain",
    "_seed": 604195
  }
]
```

### type4_village

```json
[
  {
    "phase": "pre_terrain",
    "step": 251,
    "init_dist": 45.5988,
    "raw_search_dist": 41.6885,
    "goal_corr": "mountain",
    "pos": [
      -40.2933,
      -7.7482,
      4.3002
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.4456
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.6201,
      41.6839,
      -0.4472
    ],
    "dist_xy": 41.6885,
    "agl": 4.2002,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 252,
    "init_dist": 45.5988,
    "raw_search_dist": 41.6721,
    "goal_corr": "mountain",
    "pos": [
      -40.325,
      -7.7313,
      4.3214
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.5189
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.6519,
      41.667,
      -0.4684
    ],
    "dist_xy": 41.6721,
    "agl": 4.2214,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 253,
    "init_dist": 45.5988,
    "raw_search_dist": 41.6559,
    "goal_corr": "mountain",
    "pos": [
      -40.356,
      -7.7146,
      4.3429
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.579
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.6829,
      41.6503,
      -0.4899
    ],
    "dist_xy": 41.6559,
    "agl": 4.2429,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 254,
    "init_dist": 45.5988,
    "raw_search_dist": 41.64,
    "goal_corr": "mountain",
    "pos": [
      -40.3862,
      -7.6982,
      4.3643
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.6283
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.7131,
      41.6339,
      -0.5114
    ],
    "dist_xy": 41.64,
    "agl": 4.2643,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 255,
    "init_dist": 45.5988,
    "raw_search_dist": 41.6243,
    "goal_corr": "mountain",
    "pos": [
      -40.4153,
      -7.682,
      4.3861
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.6688
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.7422,
      41.6177,
      -0.5331
    ],
    "dist_xy": 41.6243,
    "agl": 4.2861,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 256,
    "init_dist": 45.5988,
    "raw_search_dist": 41.6088,
    "goal_corr": "mountain",
    "pos": [
      -40.4434,
      -7.666,
      4.4077
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.7019
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.7702,
      41.6017,
      -0.5547
    ],
    "dist_xy": 41.6088,
    "agl": 4.3077,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 257,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5936,
    "goal_corr": "mountain",
    "pos": [
      -40.47,
      -7.6503,
      4.4294
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.7291
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.7968,
      41.586,
      -0.5764
    ],
    "dist_xy": 41.5936,
    "agl": 4.3294,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 258,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5787,
    "goal_corr": "mountain",
    "pos": [
      -40.4953,
      -7.6348,
      4.4508
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.7514
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.8221,
      41.5705,
      -0.5979
    ],
    "dist_xy": 41.5787,
    "agl": 4.3508,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 259,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5639,
    "goal_corr": "mountain",
    "pos": [
      -40.5191,
      -7.6196,
      4.4724
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.7697
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.846,
      41.5553,
      -0.6194
    ],
    "dist_xy": 41.5639,
    "agl": 4.3724,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 260,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5494,
    "goal_corr": "mountain",
    "pos": [
      -40.5415,
      -7.6047,
      4.4935
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.7847
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.8684,
      41.5404,
      -0.6405
    ],
    "dist_xy": 41.5494,
    "agl": 4.3935,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 261,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5352,
    "goal_corr": "mountain",
    "pos": [
      -40.5626,
      -7.59,
      4.5139
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.797
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.8894,
      41.5257,
      -0.6609
    ],
    "dist_xy": 41.5352,
    "agl": 4.4139,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 262,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5212,
    "goal_corr": "mountain",
    "pos": [
      -40.5824,
      -7.5755,
      4.5334
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8071
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.9092,
      41.5112,
      -0.6804
    ],
    "dist_xy": 41.5212,
    "agl": 4.4334,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 263,
    "init_dist": 45.5988,
    "raw_search_dist": 41.5074,
    "goal_corr": "mountain",
    "pos": [
      -40.601,
      -7.5613,
      4.5521
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8153
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.9279,
      41.497,
      -0.6991
    ],
    "dist_xy": 41.5074,
    "agl": 4.4521,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 264,
    "init_dist": 45.5988,
    "raw_search_dist": 41.4939,
    "goal_corr": "mountain",
    "pos": [
      -40.6186,
      -7.5474,
      4.5697
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8221
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.9454,
      41.4831,
      -0.7167
    ],
    "dist_xy": 41.4939,
    "agl": 4.4697,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 265,
    "init_dist": 45.5988,
    "raw_search_dist": 41.4805,
    "goal_corr": "mountain",
    "pos": [
      -40.6351,
      -7.5337,
      4.5864
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8277
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.9619,
      41.4694,
      -0.7334
    ],
    "dist_xy": 41.4805,
    "agl": 4.4864,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 266,
    "init_dist": 45.5988,
    "raw_search_dist": 41.4673,
    "goal_corr": "mountain",
    "pos": [
      -40.6507,
      -7.5201,
      4.6022
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8322
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.9775,
      41.4557,
      -0.7492
    ],
    "dist_xy": 41.4673,
    "agl": 4.5022,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 267,
    "init_dist": 45.5988,
    "raw_search_dist": 41.454,
    "goal_corr": "mountain",
    "pos": [
      -40.6654,
      -7.5065,
      4.6171
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.836
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      0.9922,
      41.4422,
      -0.7641
    ],
    "dist_xy": 41.454,
    "agl": 4.5171,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 268,
    "init_dist": 45.5988,
    "raw_search_dist": 41.4407,
    "goal_corr": "mountain",
    "pos": [
      -40.6794,
      -7.4928,
      4.6311
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.839
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      1.0062,
      41.4285,
      -0.7781
    ],
    "dist_xy": 41.4407,
    "agl": 4.5311,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 269,
    "init_dist": 45.5988,
    "raw_search_dist": 41.4272,
    "goal_corr": "mountain",
    "pos": [
      -40.6927,
      -7.479,
      4.6442
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8415
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      1.0195,
      41.4147,
      -0.7912
    ],
    "dist_xy": 41.4272,
    "agl": 4.5442,
    "_group": "type4_village",
    "_seed": 604197
  },
  {
    "phase": "pre_terrain",
    "step": 270,
    "init_dist": 45.5988,
    "raw_search_dist": 41.4135,
    "goal_corr": "mountain",
    "pos": [
      -40.7054,
      -7.4649,
      4.6566
    ],
    "target": [
      -39.6732,
      33.9357,
      5.853
    ],
    "target_est": [
      -39.6732,
      33.9357,
      3.8436
    ],
    "target_vel": [
      0.0,
      0.0,
      0.0
    ],
    "search_vec": [
      1.0323,
      41.4006,
      -0.8036
    ],
    "dist_xy": 41.4135,
    "agl": 4.5566,
    "_group": "type4_village",
    "_seed": 604197
  }
]
```

