MM_TO_M = 1.0 / 1000.0
M_TO_MM = 1000.0

SPEEDDATA_CONSTANTS = [
    "v5",
    "v10",
    "v20",
    "v30",
    "v40",
    "v50",
    "v60",
    "v80",
    "v100",
    "v150",
    "v200",
    "v300",
    "v400",
    "v500",
    "v600",
    "v800",
    "v1000",
    "v1500",
    "v2000",
    "v2500",
    "v3000",
    "v4000",
    "v5000",
    "v6000",
    "v7000",
    "vmax",
]

ZONEDATA_CONSTANTS = [
    "fine",
    "z0",
    "z1",
    "z5",
    "z10",
    "z15",
    "z20",
    "z30",
    "z40",
    "z50",
    "z60",
    "z80",
    "z100",
    "z150",
    "z200",
]

WAYPOINTS = [
    [
        -71.527854,
        -62.912414,
        17.982946,
        108.933682,
        75.659873,
        139.551858,
        61.097008,
    ],
    [
        -71.293095,
        -64.092921,
        19.960021,
        109.549570,
        74.551036,
        137.492666,
        61.805825,
    ],
    [
        -71.108402,
        -65.277083,
        21.859467,
        110.282613,
        73.425157,
        135.463612,
        62.586726,
    ],
    [
        -70.970395,
        -66.467481,
        23.682476,
        111.134223,
        72.287145,
        133.463423,
        63.437846,
    ],
    [
        -70.875991,
        -67.667199,
        25.429885,
        112.105618,
        71.141908,
        131.491399,
        64.357250,
    ],
    [
        -70.822399,
        -68.879775,
        27.102102,
        113.197737,
        69.994357,
        129.547495,
        65.342949,
    ],
    [
        -70.807004,
        -70.109191,
        28.699144,
        114.411105,
        68.849343,
        127.632403,
        66.392838,
    ],
    [
        -70.827271,
        -71.359864,
        30.220664,
        115.745679,
        67.711553,
        125.747645,
        67.504645,
    ],
    [
        -70.880639,
        -72.636600,
        31.665973,
        117.200685,
        66.585384,
        123.895655,
        68.675883,
    ],
    [
        -70.964428,
        -73.944522,
        33.034054,
        118.774452,
        65.474775,
        122.079850,
        69.903821,
    ],
    [
        -71.075746,
        -75.288962,
        34.323566,
        120.464238,
        64.383003,
        120.304667,
        71.185459,
    ],
    [
        -71.211412,
        -76.675306,
        35.532847,
        122.266075,
        63.312458,
        118.575548,
        72.517523,
    ],
    [
        -71.367894,
        -78.108808,
        36.659913,
        124.174635,
        62.264389,
        116.898864,
        73.896467,
    ],
    [
        -71.541280,
        -79.594362,
        37.702463,
        126.183146,
        61.238661,
        115.281762,
        75.318489,
    ],
    [
        -71.727265,
        -81.136262,
        38.657881,
        128.283352,
        60.233541,
        113.731921,
        76.779551,
    ],
    [
        -71.921195,
        -82.737959,
        39.523258,
        130.465550,
        59.245541,
        112.257241,
        78.275400,
    ],
    [
        -72.118129,
        -84.401845,
        40.295429,
        132.718694,
        58.269351,
        110.865460,
        79.801581,
    ],
    [
        -72.312951,
        -86.129082,
        40.971021,
        135.030557,
        57.297882,
        109.563750,
        81.353440,
    ],
    [
        -72.500499,
        -87.919495,
        41.546530,
        137.387951,
        56.322428,
        108.358314,
        82.926123,
    ],
    [
        -72.675710,
        -89.771537,
        42.018414,
        139.776977,
        55.332946,
        107.254034,
        84.514557,
    ],
    [
        -72.833776,
        -91.682328,
        42.383198,
        142.183297,
        54.318428,
        106.254188,
        86.113439,
    ],
    [
        -72.970285,
        -93.647754,
        42.637589,
        144.592404,
        53.267336,
        105.360283,
        87.717227,
    ],
    [
        -72.970285,
        -93.647754,
        42.637589,
        144.592404,
        53.267336,
        105.360283,
        87.717227,
    ],
    [
        -71.161828,
        -92.031235,
        43.383215,
        142.648857,
        53.594820,
        106.489447,
        86.791605,
    ],
    [
        -69.326915,
        -90.456754,
        44.036169,
        140.683402,
        53.848531,
        107.706850,
        85.806038,
    ],
    [
        -67.466573,
        -88.929558,
        44.596506,
        138.701309,
        54.035583,
        109.016359,
        84.766247,
    ],
    [
        -65.582724,
        -87.454881,
        45.064517,
        136.708274,
        54.163870,
        110.421104,
        83.678232,
    ],
    [
        -63.678217,
        -86.037863,
        45.440735,
        134.710406,
        54.241987,
        111.923320,
        82.548229,
    ],
    [
        -61.756839,
        -84.683458,
        45.725936,
        132.714212,
        54.279096,
        113.524213,
        81.382667,
    ],
    [
        -59.823293,
        -83.396322,
        45.921129,
        130.726584,
        54.284761,
        115.223832,
        80.188121,
    ],
    [
        -57.883163,
        -82.180685,
        46.027538,
        128.754776,
        54.268739,
        117.020962,
        78.971259,
    ],
    [
        -55.942828,
        -81.040207,
        46.046580,
        126.806373,
        54.240743,
        118.913054,
        77.738777,
    ],
    [
        -54.009352,
        -79.977830,
        45.979831,
        124.889240,
        54.210171,
        120.896193,
        76.497341,
    ],
    [
        -52.090333,
        -78.995629,
        45.828990,
        123.011450,
        54.185819,
        122.965123,
        75.253510,
    ],
    [
        -50.193718,
        -78.094679,
        45.595842,
        121.181174,
        54.175602,
        125.113333,
        74.013668,
    ],
    [
        -48.327591,
        -77.274943,
        45.282212,
        119.406544,
        54.186284,
        127.333206,
        72.783949,
    ],
    [
        -46.499954,
        -76.535217,
        44.889930,
        117.695480,
        54.223260,
        129.616243,
        71.570163,
    ],
    [
        -44.718504,
        -75.873113,
        44.420793,
        116.055500,
        54.290397,
        131.953326,
        70.377726,
    ],
    [
        -42.990425,
        -75.285112,
        43.876536,
        114.493520,
        54.389949,
        134.335031,
        69.211594,
    ],
    [
        -41.322227,
        -74.766684,
        43.258806,
        113.015668,
        54.522571,
        136.751949,
        68.076207,
    ],
    [
        -39.719615,
        -74.312449,
        42.569149,
        111.627121,
        54.687409,
        139.194991,
        66.975443,
    ],
    [
        -38.187428,
        -73.916390,
        41.809002,
        110.331992,
        54.882271,
        141.655662,
        65.912595,
    ],
    [
        -36.729612,
        -73.572081,
        40.979683,
        109.133268,
        55.103845,
        144.126277,
        64.890349,
    ],
    [
        -35.349257,
        -73.272916,
        40.082397,
        108.032794,
        55.347954,
        146.600110,
        63.910795,
    ],
    [
        -35.349257,
        -73.272916,
        40.082397,
        108.032794,
        55.347954,
        146.600110,
        63.910795,
    ],
    [
        -36.673925,
        -72.337238,
        39.492166,
        107.864748,
        56.033466,
        145.648911,
        63.536652,
    ],
    [
        -38.017575,
        -71.399260,
        38.843368,
        107.675051,
        56.709797,
        144.757642,
        63.199476,
    ],
    [
        -39.380325,
        -70.460525,
        38.136731,
        107.466682,
        57.376501,
        143.925520,
        62.900766,
    ],
    [
        -40.762434,
        -69.522578,
        37.373011,
        107.242498,
        58.033268,
        143.151733,
        62.641884,
    ],
    [
        -42.164302,
        -68.586966,
        36.552973,
        107.005225,
        58.679913,
        142.435446,
        62.424049,
    ],
    [
        -43.586472,
        -67.655236,
        35.677375,
        106.757452,
        59.316367,
        141.775807,
        62.248345,
    ],
    [
        -45.029626,
        -66.728938,
        34.746960,
        106.501637,
        59.942676,
        141.171958,
        62.115725,
    ],
    [
        -46.494586,
        -65.809627,
        33.762445,
        106.240116,
        60.558988,
        140.623039,
        62.027021,
    ],
    [
        -47.982312,
        -64.898869,
        32.724505,
        105.975114,
        61.165561,
        140.128198,
        61.982944,
    ],
    [
        -49.493902,
        -63.998252,
        31.633769,
        105.708756,
        61.762753,
        139.686597,
        61.984092,
    ],
    [
        -51.030585,
        -63.109391,
        30.490809,
        105.443085,
        62.351023,
        139.297419,
        62.030953,
    ],
    [
        -52.593725,
        -62.233942,
        29.296130,
        105.180085,
        62.930933,
        138.959879,
        62.123904,
    ],
    [
        -54.184811,
        -61.373615,
        28.050162,
        104.921695,
        63.503142,
        138.673227,
        62.263208,
    ],
    [
        -55.805451,
        -60.530185,
        26.753252,
        104.669840,
        64.068407,
        138.436759,
        62.449013,
    ],
    [
        -57.457367,
        -59.705505,
        25.405657,
        104.426453,
        64.627585,
        138.249822,
        62.681336,
    ],
    [
        -59.142384,
        -58.901516,
        24.007533,
        104.193501,
        65.181622,
        138.111821,
        62.960055,
    ],
    [
        -60.862414,
        -58.120263,
        22.558927,
        103.973015,
        65.731558,
        138.022228,
        63.284886,
    ],
    [
        -62.619444,
        -57.363898,
        21.059768,
        103.767116,
        66.278511,
        137.980588,
        63.655362,
    ],
    [
        -64.415510,
        -56.634688,
        19.509855,
        103.578044,
        66.823675,
        137.986529,
        64.070805,
    ],
    [
        -66.252680,
        -55.935016,
        17.908842,
        103.408184,
        67.368296,
        138.039769,
        64.530291,
    ],
    [
        -68.133024,
        -55.267374,
        16.256221,
        103.260092,
        67.913660,
        138.140130,
        65.032615,
    ],
]
