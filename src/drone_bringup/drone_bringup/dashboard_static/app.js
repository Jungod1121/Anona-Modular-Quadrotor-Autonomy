const I18N = {
  en: {
    title: 'UAV Sim Console',
    brandName: 'Flight Deck',
    brandTag: 'Mission Console',
    docTitle: 'Flight Deck · Mission Console',
    runMode: 'Mode',
    modeSingle: 'Single',
    modeMulti: 'Multi',
    planner: 'Planner',
    plannerHint: 'Pick a planner by class, then Start.',
    multiMode: 'Multi mission',
    multiHint: 'Use this page: Multi → Shared field / EGO-Swarm / Formation → Start. Shared field is fixed 2 drones on dense_field; check the command preview says shared_field.launch.py (not planner_sim).',
    numDrones: 'Drones',
    formation: 'Formation',
    map: 'Map',
    mapHint: 'Any map works with any planner (cloud_bridge). Defaults are only suggestions — pick freely.',
    mapSeed: 'Map seed',
    maxVel: 'Max vel (m/s)',
    useRviz: 'Launch RViz2 with the stack',
    start: 'Start',
    restart: 'Restart',
    stop: 'Stop',
    equivCmd: 'Equivalent command',
    live: 'Live',
    position: 'Position',
    velocity: 'Velocity',
    uptime: 'Uptime',
    warnings: 'Warnings',
    warnNone: 'None',
    warnOffline: 'Dashboard offline',
    warnFallback: 'Fallback active',
    warnPlannerFail: 'Planner FAIL',
    pid: 'PID',
    goal: 'Goal',
    goalHint: 'Publish goals in RViz with <strong>2D Goal Pose</strong>. Height uses cruise_height (default 1.0 m).',
    exploreHint: 'Path D: click <strong>Start explore</strong> (or RViz 2D Goal) to trigger the frontier FSM.',
    startExplore: 'Start explore',
    exploreStatus: 'Explore status',
    presets: 'Presets',
    presetCross: 'Corridor ±15',
    presetSwarm: 'EGO-Swarm ×2',
    presetHome: 'Dyn-A* defaults',
    presetFuel: 'Frontier explore',
    processLog: 'Process log',
    clearView: 'Clear view',
    footerPrefix: 'Local only',
    footerSuffix: 'cascade-PID plant · no SO3',
    navGuide: 'Guide',
    guideTitle: 'How to use Flight Deck',
    guideLead: 'Learn inside the app — Lab, Single, Multi, and this Guide. No need to open another page.',
    guideToc: 'On this page',
    guideTocStart: '1. Know the UI',
    guideTocChrome: '2. Background / theme / language',
    guideTocSingle: '3. Single flight',
    guideTocMulti: '4. Multi-agent',
    guideTocMap: '5. Map & top-down',
    guideTocLab: '6. Acceptance lab',
    guideTocTips: '7. Common issues',
    guideStartTitle: '1. Know the UI',
    guideStartP1: 'The left rail is mission navigation: Lab, Single, Multi, Guide. Click a page to switch tasks without leaving this window.',
    guideStartLi1: '<strong>Lab</strong>: run the six acceptance scenarios and review results (default landing page).',
    guideStartLi2: '<strong>Single</strong>: pick a planner, pick a map, set a goal, then start the sim.',
    guideStartLi3: '<strong>Multi</strong>: pick a swarm mode, drone count / formation, map, then start.',
    guideStartLi4: '<strong>Guide</strong>: this in-app how-to for Flight Deck.',
    guideChromeTitle: '2. Background / theme / language',
    guideChromeP1: 'Top-right chrome changes look only — it does not change mission config:',
    guideChromeLi1: '<strong>Background</strong>: tap a preset thumbnail, or upload a local image.',
    guideChromeLi2: '<strong>Light / Dark</strong>: day uses CSS frost; night enables LiquidGlass.',
    guideChromeLi3: '<strong>EN / 中文</strong>: switch UI language (choice is remembered).',
    guideSingleTitle: '3. Single flight',
    guideSingleS1: 'Open the <strong>Single</strong> page.',
    guideSingleS2: 'In the Planner glass panel, pick a path (A–H).',
    guideSingleS3: 'In the Map panel, pick a scene (optional seed / max speed).',
    guideSingleS4: 'On the right, press <strong>Start</strong>. Check RViz if you want the 3D view.',
    guideSingleS5: 'In RViz, publish a goal with <strong>2D Goal Pose</strong>; the top-down panel shows obstacles and tracks.',
    guideSingleS6: 'Use <strong>Stop</strong> to end; <strong>Restart</strong> to relaunch.',
    guideSingleHint: 'Tip: presets at the top of Single (Dyn-A*, corridor, …) pick planner + map for you.',
    guideMultiTitle: '4. Multi-agent',
    guideMultiS1: 'Open the <strong>Multi</strong> page.',
    guideMultiS2: 'Choose a task: EGO-Swarm / shared field / formation.',
    guideMultiS3: 'Set drone count and formation when the mode needs them.',
    guideMultiS4: 'Pick a map, then press <strong>Start</strong>.',
    guideMapTitle: '5. Map & top-down',
    guideMapP1: 'On Single / Multi, the right column is Map · top-down: obstacles, flown path, and planned path. Switch heading-up / north-up; drag the bottom edge to resize.',
    guideMapP2: 'Below that, Monitor shows pose, speed, warnings, FSM, and logs. Sim controls sit above Monitor: Stop / Restart / Start.',
    guideLabTitle: '6. Acceptance lab',
    guideLabS1: 'Open the <strong>Lab</strong> page.',
    guideLabS2: 'Check “Open RViz2 during acceptance” if you want to watch.',
    guideLabS3: 'Press <strong>Run all 6</strong>, or run one scenario from the list.',
    guideLabS4: 'Result history is on the right; the report folder is shown on the page.',
    guideTipsTitle: '7. Common issues',
    guideTipsLi1: '<strong>Start does nothing</strong>: check process logs; confirm ROS / workspace setup from the README.',
    guideTipsLi2: '<strong>Empty top-down</strong>: start the sim first and wait for the map cloud; status may say “waiting for map”.',
    guideTipsLi3: '<strong>Background won’t change</strong>: hard-refresh once; large uploads need a moment to load.',
    guideTipsLi4: '<strong>Browse only</strong>: you can explore pages and presets without flying; Start is what launches sim processes.',
    guideTipsFoot: 'This guide lives inside the console. Open Guide from the rail anytime — no second browser tab required.',
    pageHome: 'Home',
    pageSingle: 'Single',
    pageMulti: 'Multi',
    pageLab: 'Lab',
    pageTrain: 'Train',
    pageMonitor: 'Monitor',
    homeTitle: 'Flight Deck',
    homeSubtitle: 'Separate pages for single flight, swarm, and acceptance lab. Live data sits under the map.',
    homeSingleTitle: 'Single flight',
    homeSingleDesc: 'Planners A–H, map, RViz goals. Path G includes optional PPO training.',
    homeMultiTitle: 'Multi-agent',
    homeMultiDesc: 'EGO-Swarm / shared field / formation, up to 20.',
    homeLabTitle: 'Acceptance lab',
    homeLabDesc: 'Six scenarios — one or all.',
    homeMonitorDesc: 'Telemetry, diagnostics, logs, and command preview.',
    homePresetHint: 'Load a common combo and jump to the right page.',
    navSectionMain: 'Mission',
    navSectionOther: 'Other',
    navConfig: 'Setup',
    navStatus: 'Status',
    navOps: 'Ops',
    navLogs: 'Logs',
    ambianceDay: 'Light',
    ambianceNight: 'Dark',
    bgPicker: 'BG',
    bgPickerTitle: 'Background',
    bgPickerHint: 'Pick a preset or upload a local image.',
    bgUpload: 'Upload',
    bgReset: 'Default',
    bgUploadFailed: 'Background upload failed',
    bgDeleteFailed: 'Could not delete background',
    missionRunTitle: 'Sim control',
    missionRunHint: 'Start / stop the single- or multi-drone mission on this page.',
    trackTitle: 'Map · top-down',
    trackClear: 'Clear track',
    trackEmpty: 'No pose yet',
    trackPoints: 'pts',
    trackHint: 'RViz top-down · obstacles · flown + planned',
    trackWaitingMap: 'Waiting for map cloud…',
    trackHeading: 'Heading-up',
    trackNorth: 'North-up',
    idle: 'Idle',
    running: 'Running',
    offline: 'Offline',
    startFailed: 'Start failed',
    classWeak: 'Weak baseline',
    classStrong: 'Strong planner',
    classMode: 'Mission mode',
    classOptional: 'Optional / lineage',
    classMulti: 'Multi-agent',
    diffSimple: 'Simple',
    diffMedium: 'Medium',
    diffComplex: 'Complex',
    diffExtreme: 'Extreme',
    sectionDiagnostics: 'Planner diagnostics',
    sectionTraining: 'Training progress',
    sectionExperiments: 'Experiments / acceptance',
    sectionResults: 'Result history',
    diagState: 'FSM state',
    diagFallback: 'Fallback',
    diagReason: 'Reason',
    diagClearance: 'Clearance',
    diagSolve: 'Solve time',
    diagHint: 'After launch, /planner/status and diagnostics appear when ROS topics are live.',
    diagInactive: 'inactive',
    diagActive: 'active',
    expHint: 'Run the six acceptance scenarios here: all at once, or one scene at a time. Check RViz below to watch live.',
    accRunAll: 'Run all 6',
    accStop: 'Stop acceptance',
    accRunOne: 'Run',
    accUseRviz: 'Open RViz2 while testing (watch live)',
    accState: 'Acceptance',
    accScore: 'Latest score',
    accIdle: 'Idle',
    accRunning: 'Running…',
    accPass: 'PASS',
    accFail: 'FAIL',
    accRvizHint: 'RViz: Fixed Frame = map · yellow /planner/trajectory · blue /drone/path',
    accBusyHint: 'A test is running — click Stop first, or wait. Run buttons unlock when idle.',
    accNoRvizHint: 'RViz is OFF for this run. Check “Open RViz2” and click Run again to watch.',
    copyCmd: 'Copy',
    reportDir: 'Report directory',
    reportsLoading: 'Loading…',
    reportsEmpty: 'No reports yet — run acceptance or batch scripts.',
    reportsError: 'Could not load reports.',
    benchBatchTitle: 'Full seven-planner comparison',
    benchBatchHint: 'Run 7 planners × 2 maps on a closed square mission (14 trials). Forest and dense field use different square sizes.',
    benchSingleTitle: 'Single independent benchmark',
    benchSingleHint: 'Pick one planner and one map; flies that map’s closed square. Results merge into the unified comparison report (replace that cell only).',
    benchDuration: 'Evaluation time per trial (seconds)',
    benchRunAll: 'Run all 14 trials',
    benchRunSingle: 'Run selected pair',
    benchStop: 'Stop benchmark',
    benchProgress: 'Progress',
    benchState: 'Benchmark status',
    benchLatest: 'Latest summary',
    benchIdle: 'Idle',
    benchRunning: 'Running…',
    benchMapForest: 'Random Forest',
    benchMapDense: 'Dense Obstacle Field',
    benchOutputHint: 'Output: report/planner_benchmark/ (per-trial charts under charts/trials/, plus aggregate summary)',
    rlTrainHint: 'Available when Path G is selected: train the PPO local planner (target ≥95% success). Independent of the sim run.',
    sacTrainHint: 'Available when Path H is selected: continue Polar DrQ-SAC from best checkpoint (dense catalog density, honest eval ≥60 eps, target ≥90%). Opens a live monitor window. Independent of the sim run.',
    startRlTrain: 'Start training',
    stopRlTrain: 'Stop training',
    rlTrainState: 'Train status',
    rlSuccess: 'Success rate',
    rlBest: 'Best rate',
    rlTarget: 'Target',
    rlSteps: 'Timesteps',
    rlCheckpoint: 'Checkpoint',
    rlIdle: 'Idle',
    rlRunning: 'Training',
    rlDone: 'Target reached',
    rlYes: 'ready',
    rlNo: 'missing',
    homemadeLabel: 'Path A — Dyn-A* + B-spline',
    homemadeDesc: 'Dyn-A* search + B-spline on occupancy grid',
    egoLabel: 'Path B — EGO-Planner',
    egoDesc: 'ego_planner + map_generator',
    gcopterLabel: 'Path C — GCOPTER / MINCO',
    gcopterDesc: 'GCOPTER / MINCO trajectory optimization',
    fuelLabel: 'Path D — Frontier explore',
    fuelDesc: 'Fog sensing + frontier FSM + EGO backend',
    mightyLabel: 'Path E — MIGHTY / Hermite',
    mightyDesc: 'Hermite spline (mit-acl/mighty–inspired plant adapter)',
    fastLabel: 'Path F — Fast-Planner kino',
    fastDesc: 'Kino-style Fast-Planner inspired plant adapter',
    rlLabel: 'Path G — VFH+ local (PX4-style)',
    rlDesc: 'Classical polar histogram avoider → smooth yellow path (not RL)',
    sacLabel: 'Path H — Polar DrQ-SAC',
    sacDesc: 'Polar image + Soft Actor-Critic → Bézier path (VFH safety fallback)',
  },
  zh: {
    title: '无人机仿真控制台',
    brandName: '无人机仿真控制台',
    brandTag: 'Flight Deck',
    docTitle: '无人机仿真控制台 · Flight Deck',
    runMode: '模式',
    modeSingle: '单机',
    modeMulti: '多机',
    planner: '规划器',
    plannerHint: '按类别选择规划器，再点启动。',
    multiMode: '多机任务',
    multiHint: '请在本页操作：多机 → 共享空域避障 / EGO-Swarm / 编队 → 启动。共享空域固定 2 机 + 密集场；确认下方命令预览是 shared_field.launch.py（不要变成 planner_sim 单机）。',
    numDrones: '机数',
    formation: '队形',
    map: '地图',
    mapHint: '任意规划器可搭配任意地图（经 cloud_bridge）。默认地图仅为推荐，可自由组合。',
    mapSeed: '地图种子',
    maxVel: '最大速度 (m/s)',
    useRviz: '一并启动 RViz2',
    start: '启动',
    restart: '重启',
    stop: '停止',
    equivCmd: '等效命令',
    live: '实时',
    position: '位置',
    velocity: '速度',
    uptime: '运行时长',
    warnings: '告警',
    warnNone: '无',
    warnOffline: '控制台离线',
    warnFallback: '已启用回退',
    warnPlannerFail: '规划器失败',
    pid: 'PID',
    goal: '目标点',
    goalHint: '请在 RViz 用 <strong>2D Goal Pose</strong> 发布目标。高度为 cruise_height（默认 1.0 m）。',
    exploreHint: '路径 D：点击 <strong>开始探索</strong>（或 RViz 2D Goal）触发边界前沿状态机。',
    startExplore: '开始探索',
    exploreStatus: '探索状态',
    presets: '预设',
    presetCross: '走廊 ±15',
    presetSwarm: 'EGO-Swarm ×2',
    presetHome: 'Dyn-A* 默认',
    presetFuel: '边界探索',
    processLog: '进程日志',
    clearView: '清空显示',
    footerPrefix: '仅本地',
    footerSuffix: '级联 PID 被控对象 · 无 SO3',
    navGuide: '指南',
    guideTitle: '怎么用本控制台',
    guideLead: '在应用内学习：实验、单机、多机与本指南。无需打开外部文档。',
    guideToc: '本页目录',
    guideTocStart: '1. 先认识界面',
    guideTocChrome: '2. 背景 / 亮暗 / 语言',
    guideTocSingle: '3. 单机怎么飞',
    guideTocMulti: '4. 多机怎么跑',
    guideTocMap: '5. 地图与俯视',
    guideTocLab: '6. 验收实验',
    guideTocTips: '7. 常见问题',
    guideStartTitle: '1. 先认识界面',
    guideStartP1: '左侧是任务导航：实验、单机、多机、指南。点选页面即可切换任务，无需离开本窗口。',
    guideStartLi1: '<strong>实验</strong>：运行六项验收，查看结果与日志（默认首页）。',
    guideStartLi2: '<strong>单机</strong>：选择规划器与地图、设置目标后启动仿真。',
    guideStartLi3: '<strong>多机</strong>：选择协同模式、机数 / 队形、地图后启动。',
    guideStartLi4: '<strong>指南</strong>：本页说明，可随时查阅控制台用法。',
    guideChromeTitle: '2. 背景 / 亮暗 / 语言',
    guideChromeP1: '顶栏右侧可随时调整外观，不影响仿真配置：',
    guideChromeLi1: '<strong>背景</strong>：点选预设缩略图，或上传本地图片。',
    guideChromeLi2: '<strong>日间 / 夜间</strong>：日间为 CSS 毛玻璃，夜间启用液态玻璃。',
    guideChromeLi3: '<strong>EN / 中文</strong>：切换界面语言，选择会自动保存。',
    guideSingleTitle: '3. 单机怎么飞',
    guideSingleS1: '打开<strong>单机</strong>页。',
    guideSingleS2: '在「规划器」面板中选择一条路径（A–H）。',
    guideSingleS3: '在「地图」面板中选择场景（也可修改种子 / 最大速度）。',
    guideSingleS4: '在右侧点击<strong>启动</strong>。需要三维可视化时勾选 RViz。',
    guideSingleS5: '在 RViz 用 <strong>2D Goal Pose</strong> 发布目标；俯视面板会显示障碍与轨迹。',
    guideSingleS6: '结束请用<strong>停止</strong>；需要重跑请点<strong>重启</strong>。',
    guideSingleHint: '提示：单机页顶部的预设（如 Dyn-A*、走廊）会自动配置规划器与地图。',
    guideMultiTitle: '4. 多机怎么跑',
    guideMultiS1: '打开<strong>多机</strong>页。',
    guideMultiS2: '选择任务：EGO-Swarm / 共享空域避障 / 编队。',
    guideMultiS3: '设置机数与队形（若该模式需要）。',
    guideMultiS4: '选择地图后点击<strong>启动</strong>。',
    guideMapTitle: '5. 地图与俯视',
    guideMapP1: '单机 / 多机页右侧为「地图 · 俯视」：障碍、实际轨迹与规划轨迹叠加显示。可切换「机头朝上 / 北朝上」，拖动底边调整高度。',
    guideMapP2: '下方监控区显示位置、速度、告警、FSM 与日志。仿真控制条位于监控区上方：停止 / 重启 / 启动。',
    guideLabTitle: '6. 验收实验',
    guideLabS1: '打开<strong>实验</strong>页。',
    guideLabS2: '需要观看时勾选「验收时打开 RViz2」。',
    guideLabS3: '点击<strong>运行全部 6 项</strong>，或在列表中单项「运行」。',
    guideLabS4: '右侧查看结果历史；报告目录显示在页内。',
    guideTipsTitle: '7. 常见问题',
    guideTipsLi1: '<strong>点击启动无响应</strong>：查看进程日志；确认本机 ROS / 工作空间已按 README 配置。',
    guideTipsLi2: '<strong>俯视为空</strong>：请先启动仿真，等待地图点云就绪；状态会提示「等待地图」。',
    guideTipsLi3: '<strong>背景无法切换</strong>：请硬刷新一次；大图上传后稍候加载。',
    guideTipsLi4: '<strong>仅浏览界面、不飞行</strong>：仍可查看各页与预设；点击启动才会拉起仿真进程。',
    guideTipsFoot: '本指南内置于控制台。侧栏点击「指南」即可返回，无需另开浏览器页面。',
    pageHome: '总览',
    pageSingle: '单机',
    pageMulti: '多机',
    pageLab: '实验',
    pageTrain: '训练',
    pageMonitor: '监控',
    homeTitle: '无人机仿真控制台',
    homeSubtitle: '按任务分页：单机飞行、多机协同与验收实验。遥测位于地图下方。',
    homeSingleTitle: '单机飞行',
    homeSingleDesc: '路径 A–H 规划器 + 地图 + RViz 目标。选路径 G 可训练 PPO。',
    homeMultiTitle: '多机协同',
    homeMultiDesc: 'EGO-Swarm / 共享空域 / 编队，最多 20 机。',
    homeLabTitle: '验收实验',
    homeLabDesc: '六项验收场景，可单项或整批运行。',
    homeMonitorDesc: '遥测、诊断、进程日志与等效命令。',
    homePresetHint: '一键载入常用组合，并跳转到对应页面。',
    navSectionMain: '任务',
    navSectionOther: '其他',
    navConfig: '配置',
    navStatus: '状态',
    navOps: '运维',
    navLogs: '日志',
    ambianceDay: '日间',
    ambianceNight: '夜间',
    bgPicker: '背景',
    bgPickerTitle: '背景图',
    bgPickerHint: '点选预设，或上传本地图片。',
    bgUpload: '上传图片',
    bgReset: '默认',
    bgUploadFailed: '背景上传失败',
    bgDeleteFailed: '无法删除背景',
    missionRunTitle: '仿真控制',
    missionRunHint: '启动 / 停止本页的单机或多机任务。',
    trackTitle: '地图 · 俯视',
    trackClear: '清空轨迹',
    trackEmpty: '暂无位姿',
    trackPoints: '点',
    trackHint: 'RViz 俯视 · 障碍 · 实际+规划轨迹',
    trackWaitingMap: '等待地图点云…',
    trackHeading: '机头朝上',
    trackNorth: '北朝上',
    idle: '空闲',
    running: '运行中',
    offline: '离线',
    startFailed: '启动失败',
    classWeak: '弱基线',
    classStrong: '强规划器',
    classMode: '任务模式',
    classOptional: '可选对照',
    classMulti: '多机',
    diffSimple: '简单',
    diffMedium: '中等',
    diffComplex: '复杂',
    diffExtreme: '极限',
    sectionDiagnostics: '规划器诊断',
    sectionTraining: '训练进度',
    sectionExperiments: '实验 / 验收',
    sectionResults: '结果历史',
    diagState: 'FSM 状态',
    diagFallback: '回退策略',
    diagReason: '原因',
    diagClearance: '净空距离',
    diagSolve: '求解耗时',
    diagHint: '启动仿真后，若 ROS 话题可用将显示 /planner/status 与 diagnostics。',
    diagInactive: '未激活',
    diagActive: '已激活',
    expHint: '在本页直接运行六项验收：可整批执行，也可单独运行某一场景。需要观看飞行时请勾选下方 RViz。',
    accRunAll: '运行全部 6 项',
    accStop: '停止验收',
    accRunOne: '单项运行',
    accUseRviz: '验收时打开 RViz2（实时查看）',
    accState: '验收状态',
    accScore: '最近结果',
    accIdle: '空闲',
    accRunning: '运行中…',
    accPass: '通过',
    accFail: '失败',
    accRvizHint: 'RViz：Fixed Frame = map · 黄线 /planner/trajectory · 蓝线 /drone/path',
    accBusyHint: '正在运行测试 —— 请先点击「停止验收」，或等待当前项结束。「单项运行」仅在空闲时可用。',
    accNoRvizHint: '本次未启动 RViz。请勾选「验收时打开 RViz2」后再点击「单项运行」。',
    copyCmd: '复制',
    reportDir: '报告目录',
    reportsLoading: '加载中…',
    reportsEmpty: '暂无报告 — 请运行验收或批量脚本。',
    reportsError: '无法加载报告列表。',
    benchBatchTitle: '七规划器完整对比',
    benchBatchHint: '一键运行 7 个规划器 × 两张地图的封闭方阵航点任务，共 14 组；森林与密集场使用不同方阵尺寸。',
    benchSingleTitle: '单项独立测试',
    benchSingleHint: '指定一个规划器与一张地图，飞对应尺寸的封闭方阵；结果按 planner×map 写入统一综合报告（仅覆盖该格）。',
    benchDuration: '单组评测时长（秒）',
    benchRunAll: '运行全部 14 组',
    benchRunSingle: '运行所选组合',
    benchStop: '停止评测',
    benchProgress: '执行进度',
    benchState: '评测状态',
    benchLatest: '最新汇总',
    benchIdle: '空闲',
    benchRunning: '运行中…',
    benchMapForest: '随机森林',
    benchMapDense: '密集障碍场',
    benchOutputHint: '输出：report/planner_benchmark/（每组独立图在 charts/trials/，另有汇总对比图）',
    rlTrainHint: '选中路径 G 后可用：训练 PPO 局部规划器（目标成功率 ≥95%）。与仿真启动相互独立。',
    sacTrainHint: '选中路径 H 后可用：从 best 权重续训 Polar DrQ-SAC（目录级密集场、诚实评测 ≥60 局、目标 ≥90%）。开始训练会自动弹出监控窗口。与仿真启动相互独立。',
    startRlTrain: '开始训练',
    stopRlTrain: '停止训练',
    rlTrainState: '训练状态',
    rlSuccess: '当前成功率',
    rlBest: '最佳成功率',
    rlTarget: '目标',
    rlSteps: '步数',
    rlCheckpoint: '检查点',
    rlIdle: '空闲',
    rlRunning: '训练中',
    rlDone: '已达目标',
    rlYes: '已就绪',
    rlNo: '缺失',
    homemadeLabel: '路径 A — Dyn-A* + B 样条',
    homemadeDesc: 'Dyn-A* 搜索 + B 样条占用栅格优化',
    egoLabel: '路径 B — EGO-Planner',
    egoDesc: 'ego_planner + map_generator',
    gcopterLabel: '路径 C — GCOPTER / MINCO',
    gcopterDesc: 'GCOPTER / MINCO 轨迹优化',
    fuelLabel: '路径 D — 边界探索',
    fuelDesc: '未知区感知 + 边界前沿状态机 + EGO 轨迹后端',
    mightyLabel: '路径 E — MIGHTY / Hermite',
    mightyDesc: 'Hermite 样条（mit-acl/mighty 思路，机体侧适配）',
    fastLabel: '路径 F — Fast-Planner kino',
    fastDesc: 'Kino 风格 Fast-Planner 思路，机体侧适配',
    rlLabel: '路径 G — VFH+ 局部（PX4 风格）',
    rlDesc: '经典极坐标直方图避障 → 平滑轨迹（黄线可视化，非强化学习）',
    sacLabel: '路径 H — 极坐标 DrQ-SAC',
    sacDesc: '极坐标图像 + Soft Actor-Critic → Bézier 轨迹（VFH 安全回退）',
  },
};

const PLANNER_I18N = {
  homemade: { label: 'homemadeLabel', desc: 'homemadeDesc' },
  ego: { label: 'egoLabel', desc: 'egoDesc' },
  gcopter: { label: 'gcopterLabel', desc: 'gcopterDesc' },
  fuel_explore: { label: 'fuelLabel', desc: 'fuelDesc' },
  mighty: { label: 'mightyLabel', desc: 'mightyDesc' },
  fast_planner: { label: 'fastLabel', desc: 'fastDesc' },
  rl: { label: 'rlLabel', desc: 'rlDesc' },
  vfh: { label: 'rlLabel', desc: 'rlDesc' },
  sac: { label: 'sacLabel', desc: 'sacDesc' },
};

const PLANNER_CLASS_ORDER = ['weak', 'strong', 'mode', 'optional'];
const CLASS_I18N = {
  weak: 'classWeak',
  strong: 'classStrong',
  mode: 'classMode',
  optional: 'classOptional',
  multi: 'classMulti',
};

const DIFF_I18N = {
  simple: 'diffSimple',
  medium: 'diffMedium',
  complex: 'diffComplex',
  extreme: 'diffExtreme',
};

// Keep in sync with maps_catalog.DASHBOARD_MAP_IDS.
const MAP_ORDER = [
  'official_forest', 'official_perlin', 'official_posts',
  'official_maze2d', 'official_maze3d',
  'dense_field', 'narrow_corridor',
];

const MULTI_ORDER = ['ego_swarm', 'shared_field', 'formation'];

const PAGES = ['lab', 'single', 'multi', 'guide'];
const DEFAULT_PAGE = 'lab';

const state = {
  page: DEFAULT_PAGE,
  mode: 'single',
  planner: 'gcopter',
  multiMode: 'ego_swarm',
  map: 'official_forest',
  planners: {},
  plannerRegistry: [],
  plannerRegistryEn: [],
  plannerRegistryZh: [],
  mapDefaults: {},
  multiModes: {},
  maps: {},
  clearLocalLogs: false,
  lang: localStorage.getItem('drone_dash_lang') || 'zh',
  running: false,
  offline: false,
  reports: [],
};

/** Top-down XY track history (client-side only; Z ignored). */
const XY_TRACK_MAX = 800;
const XY_TRACK_MIN_DIST = 0.05;
const XY_COLORS = ['#3dd6c3', '#f0a8d0', '#7dffb2', '#ffd27a', '#9ec4ff', '#c4a8ff'];
const xyTrack = {
  series: Object.create(null), // id -> [{x,y}, ...]
  yaw: Object.create(null), // id -> radians
  pose: Object.create(null), // id -> {x,y} latest (even if track skipped)
  planned: Object.create(null), // id -> [{x,y}, ...] planned path
  occupancy: null,
  headingUp: true,
  primaryId: 'main',
  _drawPending: false,
};

const el = (id) => document.getElementById(id);
const t = (key) => (I18N[state.lang] && I18N[state.lang][key]) || I18N.en[key] || key;

function clearXyTrack() {
  xyTrack.series = Object.create(null);
  xyTrack.yaw = Object.create(null);
  xyTrack.pose = Object.create(null);
  xyTrack.planned = Object.create(null);
  scheduleDrawXyTrack();
  const hint = el('xyTrackHint');
  if (hint) hint.textContent = t('trackEmpty');
}

function scheduleDrawXyTrack() {
  if (xyTrack._drawPending) return;
  xyTrack._drawPending = true;
  requestAnimationFrame(() => {
    xyTrack._drawPending = false;
    drawXyTrack();
  });
}

function setMapOrientation(headingUp) {
  xyTrack.headingUp = !!headingUp;
  const btnH = el('btnMapHeading');
  const btnN = el('btnMapNorth');
  if (btnH) btnH.classList.toggle('is-active', xyTrack.headingUp);
  if (btnN) btnN.classList.toggle('is-active', !xyTrack.headingUp);
  try {
    localStorage.setItem('drone_dash_map_heading', xyTrack.headingUp ? '1' : '0');
  } catch (e) { /* ignore */ }
  scheduleDrawXyTrack();
}

function pushXySample(id, x, y, yaw) {
  if (!Number.isFinite(x) || !Number.isFinite(y)) return;
  xyTrack.pose[id] = { x, y };
  if (Number.isFinite(yaw)) xyTrack.yaw[id] = yaw;
  if (!xyTrack.series[id]) xyTrack.series[id] = [];
  const buf = xyTrack.series[id];
  const last = buf[buf.length - 1];
  if (last) {
    const dx = x - last.x;
    const dy = y - last.y;
    if (dx * dx + dy * dy < XY_TRACK_MIN_DIST * XY_TRACK_MIN_DIST) return;
  }
  buf.push({ x, y });
  if (buf.length > XY_TRACK_MAX) buf.splice(0, buf.length - XY_TRACK_MAX);
}

function ingestXyFromStatus(st) {
  // Prefer swarm tracks whenever any uav* odom is live (do not gate on
  // state.mode — otherwise a stale "single" mode drops all but /drone/odom).
  const swarm = st.swarm_odom || {};
  const keys = Object.keys(swarm).sort();
  if (keys.length) {
    xyTrack.primaryId = keys[0];
    keys.forEach((k) => {
      const p = swarm[k];
      if (p) pushXySample(k, p.x, p.y, p.yaw);
    });
  } else if (st.odom) {
    xyTrack.primaryId = 'main';
    pushXySample('main', st.odom.x, st.odom.y, st.odom.yaw);
  }

  const nextPlanned = Object.create(null);
  if (st.planned_path && st.planned_path.length) {
    nextPlanned.main = st.planned_path;
  }
  const swarmPlan = st.swarm_planned || {};
  Object.keys(swarmPlan).forEach((k) => {
    if (swarmPlan[k] && swarmPlan[k].length) nextPlanned[k] = swarmPlan[k];
  });
  xyTrack.planned = nextPlanned;
  scheduleDrawXyTrack();
}

function worldBounds() {
  const ids = Object.keys(xyTrack.series);
  const allPts = ids.flatMap((id) => xyTrack.series[id] || []);
  const plannedPts = Object.keys(xyTrack.planned).flatMap((id) => xyTrack.planned[id] || []);
  const occ = xyTrack.occupancy;
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  if (occ && occ.width > 0 && occ.height > 0) {
    minX = occ.origin.x;
    minY = occ.origin.y;
    maxX = occ.origin.x + occ.width * occ.resolution;
    maxY = occ.origin.y + occ.height * occ.resolution;
  }
  allPts.concat(plannedPts).forEach((p) => {
    minX = Math.min(minX, p.x);
    maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y);
    maxY = Math.max(maxY, p.y);
  });
  Object.keys(xyTrack.pose).forEach((id) => {
    const p = xyTrack.pose[id];
    if (!p) return;
    minX = Math.min(minX, p.x);
    maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y);
    maxY = Math.max(maxY, p.y);
  });
  if (!Number.isFinite(minX)) {
    return { minX: -5, maxX: 5, minY: -5, maxY: 5 };
  }
  if (maxX - minX < 1) {
    minX -= 0.5;
    maxX += 0.5;
  }
  if (maxY - minY < 1) {
    minY -= 0.5;
    maxY += 0.5;
  }
  const padX = Math.max(0.5, (maxX - minX) * 0.06);
  const padY = Math.max(0.5, (maxY - minY) * 0.06);
  return {
    minX: minX - padX,
    maxX: maxX + padX,
    minY: minY - padY,
    maxY: maxY + padY,
  };
}

function primaryPose() {
  const id = xyTrack.primaryId || 'main';
  if (xyTrack.pose[id]) return { id, ...xyTrack.pose[id], yaw: xyTrack.yaw[id] || 0 };
  const ids = Object.keys(xyTrack.pose);
  if (ids.length) {
    const k = ids[0];
    return { id: k, ...xyTrack.pose[k], yaw: xyTrack.yaw[k] || 0 };
  }
  const seriesIds = Object.keys(xyTrack.series);
  for (let i = 0; i < seriesIds.length; i++) {
    const pts = xyTrack.series[seriesIds[i]];
    if (pts && pts.length) {
      const p = pts[pts.length - 1];
      return { id: seriesIds[i], x: p.x, y: p.y, yaw: xyTrack.yaw[seriesIds[i]] || 0 };
    }
  }
  return null;
}

function worldToBody(wx, wy, ax, ay, yaw) {
  const dx = wx - ax;
  const dy = wy - ay;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  // Body: +x forward, +y left
  return {
    bx: c * dx + s * dy,
    by: -s * dx + c * dy,
  };
}

function drawDroneMarker(ctx, cx, cy, dpr, night, color) {
  const s = 9 * dpr;
  ctx.beginPath();
  ctx.moveTo(cx, cy - s);
  ctx.lineTo(cx + s * 0.72, cy + s * 0.78);
  ctx.lineTo(cx, cy + s * 0.35);
  ctx.lineTo(cx - s * 0.72, cy + s * 0.78);
  ctx.closePath();
  ctx.fillStyle = color || (night ? '#7dffb2' : '#0d7a4a');
  ctx.fill();
  ctx.strokeStyle = night ? '#0a1524' : '#ffffff';
  ctx.lineWidth = 1.4 * dpr;
  ctx.stroke();
}

function droneShortLabel(id) {
  const m = String(id || '').match(/^uav(\d+)$/i);
  if (m) return `uav${m[1]}`;
  return String(id || 'drone');
}

function drawXyTrack() {
  const canvas = el('xyTrackCanvas');
  if (!canvas || canvas.offsetParent === null) return;
  const hint = el('xyTrackHint');
  const ids = Object.keys(xyTrack.series);
  const allPts = ids.flatMap((id) => xyTrack.series[id] || []);
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 512;
  const cssH = canvas.clientHeight || 320;
  const w = Math.max(1, Math.round(cssW * dpr));
  const h = Math.max(1, Math.round(cssH * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext('2d');
  const night = document.documentElement.getAttribute('data-ambiance') !== 'day';
  ctx.fillStyle = night ? '#2b3038' : '#dfe6ef';
  ctx.fillRect(0, 0, w, h);

  const bounds = worldBounds();
  const pad = 10 * dpr;
  const anchor = primaryPose();
  const headingUp = xyTrack.headingUp && !!anchor;

  let toCanvas;
  let spanLabel = '';

  if (headingUp) {
    const yaw = Number.isFinite(anchor.yaw) ? anchor.yaw : 0;
    const content = [];
    allPts.forEach((p) => content.push(worldToBody(p.x, p.y, anchor.x, anchor.y, yaw)));
    Object.keys(xyTrack.planned).forEach((id) => {
      (xyTrack.planned[id] || []).forEach((p) => {
        content.push(worldToBody(p.x, p.y, anchor.x, anchor.y, yaw));
      });
    });
    Object.keys(xyTrack.pose).forEach((id) => {
      const p = xyTrack.pose[id];
      if (p) content.push(worldToBody(p.x, p.y, anchor.x, anchor.y, yaw));
    });
    const occ = xyTrack.occupancy;
    if (occ && occ.width && occ.height) {
      const corners = [
        [occ.origin.x, occ.origin.y],
        [occ.origin.x + occ.width * occ.resolution, occ.origin.y],
        [occ.origin.x, occ.origin.y + occ.height * occ.resolution],
        [occ.origin.x + occ.width * occ.resolution, occ.origin.y + occ.height * occ.resolution],
      ];
      corners.forEach(([wx, wy]) => content.push(worldToBody(wx, wy, anchor.x, anchor.y, yaw)));
    }
    let minBx = -4;
    let maxBx = 4;
    let minBy = -4;
    let maxBy = 4;
    if (content.length) {
      minBx = Math.min(...content.map((p) => p.bx));
      maxBx = Math.max(...content.map((p) => p.bx));
      minBy = Math.min(...content.map((p) => p.by));
      maxBy = Math.max(...content.map((p) => p.by));
    }
    // Keep drone near center with at least some look-ahead
    minBx = Math.min(minBx, -2);
    maxBx = Math.max(maxBx, 6);
    minBy = Math.min(minBy, -4);
    maxBy = Math.max(maxBy, 4);
    const padB = 0.8;
    minBx -= padB;
    maxBx += padB;
    minBy -= padB;
    maxBy += padB;
    const spanX = Math.max(1, maxBx - minBx);
    const spanY = Math.max(1, maxBy - minBy);
    const sx = (w - 2 * pad) / spanY; // body Y (left/right) → screen X
    const sy = (h - 2 * pad) / spanX; // body X (forward) → screen Y
    // Uniform-ish fit so circles stay round-ish, but allow rectangular canvas
    const scale = Math.min(sx, sy);
    const ox = (w - spanY * scale) / 2;
    const oy = (h - spanX * scale) / 2;
    toCanvas = (wx, wy) => {
      const b = worldToBody(wx, wy, anchor.x, anchor.y, yaw);
      return {
        // body +y is left → screen left; body +x forward → screen up
        cx: ox + (maxBy - b.by) * scale,
        cy: h - (oy + (b.bx - minBx) * scale),
      };
    };
    // Map ground fill
    ctx.fillStyle = night ? '#1e232b' : '#cfd8e4';
    ctx.fillRect(ox, oy, spanY * scale, spanX * scale);
    spanLabel = `Hdg ${(yaw * 180 / Math.PI).toFixed(0)}°`;
  } else {
    const spanX = Math.max(1, bounds.maxX - bounds.minX);
    const spanY = Math.max(1, bounds.maxY - bounds.minY);
    const sx = (w - 2 * pad) / spanX;
    const sy = (h - 2 * pad) / spanY;
    const scale = Math.min(sx, sy);
    const ox = (w - spanX * scale) / 2;
    const oy = (h - spanY * scale) / 2;
    toCanvas = (x, y) => ({
      cx: ox + (x - bounds.minX) * scale,
      cy: h - (oy + (y - bounds.minY) * scale),
    });
    ctx.fillStyle = night ? '#1e232b' : '#cfd8e4';
    ctx.fillRect(ox, oy, spanX * scale, spanY * scale);
    spanLabel = `X ${bounds.minX.toFixed(1)}…${bounds.maxX.toFixed(1)}  Y ${bounds.minY.toFixed(1)}…${bounds.maxY.toFixed(1)}`;
  }

  const occ = xyTrack.occupancy;
  const occupied = occ && (occ.occupied || (occ.data
    ? occ.data.reduce((acc, v, i) => { if (v >= 50) acc.push(i); return acc; }, [])
    : null));
  if (occ && occupied && occ.width && occ.height) {
    const res = occ.resolution;
    // Estimate cell size from adjacent world points
    const a = toCanvas(occ.origin.x, occ.origin.y);
    const b = toCanvas(occ.origin.x + res, occ.origin.y);
    const c = toCanvas(occ.origin.x, occ.origin.y + res);
    const cell = Math.max(
      1.2 * dpr,
      Math.hypot(b.cx - a.cx, b.cy - a.cy),
      Math.hypot(c.cx - a.cx, c.cy - a.cy),
    );
    ctx.fillStyle = night ? '#e8eef6' : '#243044';
    ctx.strokeStyle = night ? 'rgba(12, 18, 28, 0.72)' : 'rgba(255, 255, 255, 0.55)';
    ctx.lineWidth = Math.max(0.8 * dpr, cell * 0.08);
    for (let k = 0; k < occupied.length; k++) {
      const idx = occupied[k];
      const ix = idx % occ.width;
      const iy = (idx / occ.width) | 0;
      const wx = occ.origin.x + (ix + 0.5) * res;
      const wy = occ.origin.y + (iy + 0.5) * res;
      const p = toCanvas(wx, wy);
      const x = p.cx - cell / 2;
      const y = p.cy - cell / 2;
      const s = cell + 0.5;
      ctx.fillRect(x, y, s, s);
      ctx.strokeRect(x + 0.4 * dpr, y + 0.4 * dpr, s - 0.8 * dpr, s - 0.8 * dpr);
    }
  }

  // Light grid
  ctx.strokeStyle = night ? 'rgba(170,190,220,0.14)' : 'rgba(40,60,90,0.16)';
  ctx.lineWidth = 1 * dpr;
  const steps = 8;
  for (let i = 1; i < steps; i++) {
    const t01 = i / steps;
    ctx.beginPath();
    ctx.moveTo(w * t01, pad);
    ctx.lineTo(w * t01, h - pad);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(pad, h * t01);
    ctx.lineTo(w - pad, h * t01);
    ctx.stroke();
  }

  ctx.fillStyle = night ? 'rgba(210,226,245,0.7)' : 'rgba(30,48,72,0.7)';
  ctx.font = `${10 * dpr}px ui-monospace, monospace`;
  ctx.textAlign = 'left';
  ctx.fillText(spanLabel, pad, h - 6 * dpr);

  if (!allPts.length && !anchor) {
    const planIds0 = Object.keys(xyTrack.planned);
    const hasPlan0 = planIds0.some((id) => (xyTrack.planned[id] || []).length);
    if (!hasPlan0) {
      ctx.textAlign = 'center';
      ctx.fillStyle = night ? 'rgba(210,226,245,0.55)' : 'rgba(30,48,72,0.55)';
      const nOcc = occupied ? occupied.length : 0;
      const emptyMsg = nOcc ? t('trackEmpty') : (occ ? t('trackEmpty') : t('trackWaitingMap'));
      ctx.fillText(emptyMsg, w / 2, h / 2);
      if (hint) {
        hint.textContent = nOcc ? `${nOcc} obs · ${t('trackEmpty')}` : emptyMsg;
      }
      return;
    }
  }

  // Planned trajectory (RViz yellow) — dashed, under flown track
  const PLAN_COLORS = ['#ffd27a', '#f0c36a', '#e8b84a', '#ffc857'];
  Object.keys(xyTrack.planned).forEach((id, idx) => {
    const pts = xyTrack.planned[id];
    if (!pts || pts.length < 2) return;
    ctx.save();
    ctx.strokeStyle = PLAN_COLORS[idx % PLAN_COLORS.length];
    ctx.lineWidth = 2.4 * dpr;
    ctx.setLineDash([7 * dpr, 5 * dpr]);
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.globalAlpha = 0.95;
    ctx.beginPath();
    pts.forEach((p, i) => {
      const c = toCanvas(p.x, p.y);
      if (i === 0) ctx.moveTo(c.cx, c.cy);
      else ctx.lineTo(c.cx, c.cy);
    });
    ctx.stroke();
    ctx.restore();
  });

  // Stable colors by sorted id so track + marker match.
  const colorIds = Array.from(new Set([
    ...Object.keys(xyTrack.pose),
    ...ids,
  ])).sort();
  const colorOf = (id) => {
    const i = colorIds.indexOf(id);
    return XY_COLORS[(i >= 0 ? i : 0) % XY_COLORS.length];
  };

  ids.forEach((id) => {
    const pts = xyTrack.series[id];
    if (!pts || pts.length < 1) return;
    const color = colorOf(id);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2 * dpr;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    pts.forEach((p, i) => {
      const c = toCanvas(p.x, p.y);
      if (i === 0) ctx.moveTo(c.cx, c.cy);
      else ctx.lineTo(c.cx, c.cy);
    });
    ctx.stroke();
  });

  // One chevron + name per drone (including the heading/primary craft).
  const markerIds = colorIds;
  const primaryYaw = anchor && Number.isFinite(anchor.yaw) ? anchor.yaw : 0;
  markerIds.forEach((id) => {
    const pose = xyTrack.pose[id]
      || (xyTrack.series[id] && xyTrack.series[id].length
        ? xyTrack.series[id][xyTrack.series[id].length - 1]
        : null);
    if (!pose || !Number.isFinite(pose.x) || !Number.isFinite(pose.y)) return;
    const color = colorOf(id);
    const tip = toCanvas(pose.x, pose.y);
    const yaw = Number.isFinite(xyTrack.yaw[id]) ? xyTrack.yaw[id] : 0;
    ctx.save();
    ctx.translate(tip.cx, tip.cy);
    if (headingUp) {
      // Map already rotated by -primaryYaw; rotate marker by relative yaw.
      ctx.rotate(-(yaw - primaryYaw));
    } else {
      ctx.rotate(-yaw + Math.PI / 2);
    }
    drawDroneMarker(ctx, 0, 0, dpr, night, color);
    ctx.restore();
    ctx.fillStyle = color;
    ctx.font = `${10 * dpr}px ui-monospace, monospace`;
    ctx.textAlign = 'left';
    ctx.fillText(droneShortLabel(id), tip.cx + 8 * dpr, tip.cy - 8 * dpr);
  });

  const n = allPts.length;
  const last = anchor || (allPts.length ? allPts[allPts.length - 1] : null);
  if (hint) {
    const nOcc = occupied ? occupied.length : 0;
    const nPlan = Object.keys(xyTrack.planned).reduce(
      (acc, id) => acc + ((xyTrack.planned[id] || []).length ? 1 : 0), 0);
    const bits = [];
    if (markerIds.length > 1) bits.push(`${markerIds.length} drones`);
    if (nOcc) bits.push(`${nOcc} obs`);
    if (nPlan) bits.push(`${nPlan} plan`);
    if (last) bits.push(`${last.x.toFixed(2)}, ${last.y.toFixed(2)} · ${n} ${t('trackPoints')}`);
    else if (!bits.length) bits.push(t('trackEmpty'));
    hint.textContent = bits.join(' · ');
  }
  if (typeof window.markMapLiquidChanged === 'function') {
    window.markMapLiquidChanged();
  }
}

async function pollOccupancy() {
  if (state.page !== 'single' && state.page !== 'multi') return;
  try {
    const data = await api('/api/map/occupancy');
    if (data && data.ok && data.occupancy) {
      xyTrack.occupancy = data.occupancy;
      scheduleDrawXyTrack();
    }
  } catch (e) {
    /* map may not be up yet */
  }
}

async function pollMapLive() {
  if (state.page !== 'single' && state.page !== 'multi') return;
  try {
    const data = await api('/api/map/live');
    if (!data || !data.ok) return;
    ingestXyFromStatus(data);
  } catch (e) {
    /* backend may be restarting */
  }
}

function normPlanner(id) {
  return id === 'rl' ? 'vfh' : id;
}

function samePlanner(a, b) {
  return normPlanner(a) === normPlanner(b);
}

function plannerUiId(id) {
  return normPlanner(id);
}

async function api(path, opts = {}) {
  const res = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...opts,
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

function applyI18n() {
  document.documentElement.lang = state.lang === 'zh' ? 'zh-CN' : 'en';
  document.querySelectorAll('[data-i18n]').forEach((node) => {
    node.textContent = t(node.dataset.i18n);
  });
  document.querySelectorAll('[data-i18n-html]').forEach((node) => {
    node.innerHTML = t(node.dataset.i18nHtml);
  });
  // Path G/H share one train card — refresh planner-specific hint after i18n.
  if (typeof updateRlUI === 'function') updateRlUI();
  document.querySelectorAll('.lang-btn').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.lang === state.lang);
    btn.classList.toggle('is-active', btn.dataset.lang === state.lang);
  });
  document.querySelectorAll('[data-ambiance]').forEach((btn) => {
    if (!btn.classList.contains('seg__btn') && !btn.classList.contains('segment__btn')) return;
    btn.classList.toggle('is-active', btn.dataset.ambiance === getAmbiance());
  });
  document.title = t('docTitle');
  setRunningUI(state.running);
  const accList = el('accScenarioList');
  if (accList) {
    delete accList.dataset.built;
    delete accList.dataset.lang;
  }
  renderPlannersForLang();
  if (Object.keys(state.multiModes).length) renderMulti(state.multiModes);
  if (Object.keys(state.maps).length) renderMaps(state.maps);
  renderReports(state.reports);
  updateModeUI();
  if (typeof showPage === 'function' && state.page) {
    const title = el('pageTitle');
    if (title) {
      const key = {
        lab: 'homeLabTitle',
        single: 'homeSingleTitle',
        multi: 'homeMultiTitle',
        guide: 'guideTitle',
      }[state.page] || 'homeLabTitle';
      title.textContent = t(key);
    }
  }
  drawXyTrack();
}

function setLang(lang) {
  if (!I18N[lang]) return;
  state.lang = lang;
  localStorage.setItem('drone_dash_lang', lang);
  applyI18n();
}

function getAmbiance() {
  const cur = document.documentElement.getAttribute('data-ambiance');
  return cur === 'night' ? 'night' : 'day';
}

function setAmbiance(amb) {
  const next = amb === 'night' ? 'night' : 'day';
  document.documentElement.setAttribute('data-ambiance', next);
  localStorage.setItem('drone_dash_ambiance', next);
  document.querySelectorAll('.seg__btn[data-ambiance], .segment__btn[data-ambiance]').forEach((btn) => {
    btn.classList.toggle('is-active', btn.dataset.ambiance === next);
  });
  document.dispatchEvent(new CustomEvent('drone-ambiance-changed', { detail: { ambiance: next } }));
}

const BG_DEFAULT = '/bg-15-sequoia-sunrise.png';
const BG_STORAGE_KEY = 'drone_dash_bg';
let bgCatalog = [];

function getSavedBackground() {
  try {
    const v = localStorage.getItem(BG_STORAGE_KEY);
    if (v && /^(\/|https?:|data:)/.test(v)) return v;
  } catch (e) { /* ignore */ }
  return BG_DEFAULT;
}

function bgUrlsMatch(a, b) {
  try {
    return new URL(a, location.origin).href === new URL(b, location.origin).href;
  } catch (e) {
    return a === b;
  }
}

function applyBackground(url, { persist = true } = {}) {
  const src = url || BG_DEFAULT;
  const img = el('appBg');
  if (img) {
    const done = () => {
      document.dispatchEvent(new CustomEvent('drone-bg-changed', { detail: { url: src } }));
    };
    if (img.complete && img.naturalWidth > 0 && bgUrlsMatch(img.src, src)) {
      done();
    } else {
      img.addEventListener('load', done, { once: true });
      img.addEventListener('error', done, { once: true });
      img.src = src;
    }
  }
  if (persist) {
    try { localStorage.setItem(BG_STORAGE_KEY, src); } catch (e) { /* ignore */ }
  }
  renderBgPickerGrid();
}

async function loadBackgroundCatalog() {
  try {
    const data = await api('/api/backgrounds');
    if (data && data.ok && Array.isArray(data.items)) {
      bgCatalog = data.items;
    }
  } catch (e) {
    bgCatalog = [{
      id: 'builtin:bg-15-sequoia-sunrise.png',
      name: '15 sequoia sunrise',
      url: BG_DEFAULT,
      builtin: true,
      deletable: false,
    }];
  }
  renderBgPickerGrid();
}

function renderBgPickerGrid() {
  const grid = el('bgPickerGrid');
  if (!grid) return;
  const current = getSavedBackground();
  grid.innerHTML = '';
  for (const item of bgCatalog) {
    const wrap = document.createElement('div');
    wrap.className = 'bg-thumb-wrap';
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'bg-thumb' + (item.url === current ? ' is-active' : '');
    btn.title = item.name || item.url;
    btn.innerHTML = `
      <img src="${item.url}" alt="" loading="lazy" decoding="async" />
      <span class="bg-thumb__label">${item.name || 'bg'}</span>`;
    btn.addEventListener('click', () => {
      applyBackground(item.url);
      setBgPickerOpen(false);
    });
    wrap.appendChild(btn);
    if (item.deletable) {
      const del = document.createElement('button');
      del.type = 'button';
      del.className = 'bg-thumb__del';
      del.title = 'Delete';
      del.setAttribute('aria-label', 'Delete');
      del.textContent = '×';
      del.addEventListener('click', (ev) => {
        ev.stopPropagation();
        deleteBackground(item.id).catch((err) => alert(err.message || t('bgDeleteFailed')));
      });
      wrap.appendChild(del);
    }
    grid.appendChild(wrap);
  }
}

function setBgPickerOpen(open) {
  const panel = el('bgPickerPanel');
  const toggle = el('btnBgPicker');
  const root = el('bgPicker');
  if (!panel || !toggle) return;
  panel.hidden = !open;
  toggle.setAttribute('aria-expanded', open ? 'true' : 'false');
  toggle.classList.toggle('is-open', open);
  if (root) root.classList.toggle('is-open', open);
  if (open) loadBackgroundCatalog();
}

async function uploadBackgroundFile(file) {
  if (!file) return;
  const dataUrl = await new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(String(reader.result || ''));
    reader.onerror = () => reject(new Error(t('bgUploadFailed')));
    reader.readAsDataURL(file);
  });
  const data = await api('/api/backgrounds', {
    method: 'POST',
    body: JSON.stringify({ filename: file.name, data: dataUrl }),
  });
  if (!data.ok) throw new Error(data.error || t('bgUploadFailed'));
  if (Array.isArray(data.items)) bgCatalog = data.items;
  if (data.item && data.item.url) applyBackground(data.item.url);
  else renderBgPickerGrid();
}

async function deleteBackground(id) {
  const data = await api('/api/backgrounds/delete', {
    method: 'POST',
    body: JSON.stringify({ id }),
  });
  if (!data.ok) throw new Error(data.error || t('bgDeleteFailed'));
  if (Array.isArray(data.items)) bgCatalog = data.items;
  const gone = !bgCatalog.some((it) => it.url === getSavedBackground());
  if (gone) applyBackground(BG_DEFAULT);
  else renderBgPickerGrid();
}

function bindBackgroundPicker() {
  const toggle = el('btnBgPicker');
  const panel = el('bgPickerPanel');
  const root = el('bgPicker');
  const fileInput = el('bgFileInput');
  const reset = el('btnBgReset');
  if (!toggle || !panel || !root) return;

  toggle.addEventListener('click', (ev) => {
    ev.preventDefault();
    ev.stopPropagation();
    setBgPickerOpen(panel.hidden);
  });
  // Close only when clicking outside the whole picker (not on open toggle).
  document.addEventListener('click', (ev) => {
    if (panel.hidden) return;
    if (root.contains(ev.target)) return;
    setBgPickerOpen(false);
  });
  document.addEventListener('keydown', (ev) => {
    if (ev.key === 'Escape') setBgPickerOpen(false);
  });

  if (fileInput) {
    fileInput.addEventListener('change', () => {
      const file = fileInput.files && fileInput.files[0];
      fileInput.value = '';
      if (!file) return;
      uploadBackgroundFile(file).catch((err) => alert(err.message || t('bgUploadFailed')));
    });
  }
  if (reset) {
    reset.addEventListener('click', () => {
      applyBackground(BG_DEFAULT);
      setBgPickerOpen(false);
    });
  }

  // Ensure saved choice is applied even if early inline script missed.
  applyBackground(getSavedBackground(), { persist: false });
  loadBackgroundCatalog();
}

function markLiquidCard(btn) {
  // Option cards stay nested CSS frost inside a parent LiquidGlass panel
  // (same pattern as map cards inside #mapPanel). Do not promote them.
  btn.classList.remove('liquid-card', 'mission-lg__stack', 'is-liquid-glass');
  delete btn.dataset.liquid;
  delete btn.dataset.liquidRole;
  btn.removeAttribute('data-liquid');
  btn.removeAttribute('data-liquid-role');
}

/** @deprecated Hoisting option cards broke the map-panel pattern; kept as no-op cleaner. */
function placeAfter(lg, anchor, nodes) {
  if (!lg || !nodes.length) return;
  const frag = document.createDocumentFragment();
  for (const node of nodes) frag.appendChild(node);
  lg.insertBefore(frag, anchor ? anchor.nextSibling : null);
}

function clearHoistedOptionCards() {
  document.querySelectorAll(
    '#missionLgSingle > .planner-card, #missionLgSingle > .map-card, #missionLgSingle > .planner-group__title.liquid-hoist,'
    + '#missionLgMulti > .planner-card, #missionLgMulti > .map-card, #missionLgMulti > .planner-group__title.liquid-hoist',
  ).forEach((n) => n.remove());
}

function refreshLiquidCards() {
  const page = document.documentElement.getAttribute('data-page') || state.page;
  // Only re-init when the mission pages are visible. Background data loads
  // on 首页 must not destroy home liquid glass.
  if (page !== 'single' && page !== 'multi' && page !== 'lab') return;
  if (typeof window.refreshMissionLiquid === 'function') {
    window.refreshMissionLiquid();
  }
}

function mountMapPanel(page) {
  const panelHost = el('mapPanelHost');
  const fieldsHost = el('mapFieldsHost');
  const trackHost = el('xyTrackHost');
  const panel = el('mapPanel');
  const fields = el('mapFields');
  const track = el('xyTrackPanel');
  const lg = page === 'single' ? el('missionLgSingle')
    : page === 'multi' ? el('missionLgMulti') : null;

  if (lg) {
    const mountHost = (host, beforeGoal) => {
      if (!host) return;
      host.hidden = false;
      const goal = lg.querySelector('[data-liquid-slot="goal"]');
      if (host.parentElement !== lg) {
        if (beforeGoal && goal) lg.insertBefore(host, goal);
        else lg.appendChild(host);
      } else if (beforeGoal && goal && host.nextElementSibling !== goal
          && host !== goal.previousElementSibling) {
        // Keep map picker / fields ahead of goal when already in tree.
        if (!host.contains(goal)) lg.insertBefore(host, goal);
      }
    };

    if (panel) {
      panel.className = 'mission-shell liquid-panel--main map-panel map-panel--calc';
      panel.dataset.liquid = '1';
      panel.dataset.liquidRole = 'panel';
      panel.setAttribute('data-no-drag', '');
    }
    if (fields) {
      fields.className = 'mission-shell liquid-panel--main map-panel-fields map-panel--calc';
      fields.dataset.liquid = '1';
      fields.dataset.liquidRole = 'panel';
      fields.setAttribute('data-no-drag', '');
    }
    if (track) {
      track.classList.add('frost-panel', 'liquid-panel--map', 'map-panel--calc');
      track.classList.remove('liquid-panel', 'mission-shell');
      track.dataset.liquid = '1';
      track.dataset.liquidRole = 'panel';
      track.setAttribute('data-no-drag', '');
    }
    [panelHost, fieldsHost, trackHost].forEach((host) => {
      if (host) host.classList.add('mission-feather');
    });

    // Track first so it shares row 1 with the lead left shell (top-aligned).
    mountHost(trackHost, false);
    if (trackHost && trackHost.parentElement === lg && lg.firstElementChild !== trackHost) {
      lg.insertBefore(trackHost, lg.firstElementChild);
    }
    mountHost(panelHost, true);
    mountHost(fieldsHost, true);

    if (track) {
      requestAnimationFrame(() => drawXyTrack());
      pollOccupancy();
      pollMapLive();
    }
    if (Object.keys(state.maps || {}).length) placeMapCards(lg);

    if (typeof window.refreshMapLiquid === 'function') {
      requestAnimationFrame(() => window.refreshMapLiquid());
    }
  } else {
    if (panelHost) panelHost.hidden = true;
    if (fieldsHost) fieldsHost.hidden = true;
    if (trackHost) trackHost.hidden = true;
  }
}

function showPage(name, opts = {}) {
  // Legacy bookmarks — home / train / monitor redirect.
  if (name === 'home' || name === 'train' || name === 'monitor') name = name === 'home' ? DEFAULT_PAGE : 'single';

  const page = PAGES.includes(name) ? name : DEFAULT_PAGE;
  state.page = page;
  document.documentElement.setAttribute('data-page', page);
  document.querySelectorAll('.page').forEach((node) => {
    node.classList.toggle('is-active', node.dataset.page === page);
  });
  document.querySelectorAll('[data-nav]').forEach((btn) => {
    btn.classList.toggle('is-active', btn.dataset.nav === page);
  });
  const chip = el('pageChip');
  if (chip) chip.textContent = page.toUpperCase();
  const title = el('pageTitle');
  if (title) {
    const key = {
      lab: 'homeLabTitle',
      single: 'homeSingleTitle',
      multi: 'homeMultiTitle',
      guide: 'guideTitle',
    }[page] || 'homeLabTitle';
    title.textContent = t(key);
  }
  mountMapPanel(page);
  placeMissionRunBar(page);
  placeMonitorDock(page);

  if (page === 'single' && state.mode !== 'single') {
    selectRunMode('single', opts.sync !== false);
  } else if (page === 'multi' && state.mode !== 'multi') {
    selectRunMode('multi', opts.sync !== false);
  } else {
    updateModeUI();
  }

  updateRlUI();

  try {
    localStorage.setItem('drone_dash_page', page);
  } catch (e) { /* ignore */ }
  if (location.hash.replace(/^#/, '') !== page) {
    history.replaceState(null, '', `#${page}`);
  }
  document.dispatchEvent(new CustomEvent('drone-page-changed', { detail: { page } }));
}

function updateModeUI() {
  document.querySelectorAll('.mode-chip').forEach((b) => {
    const on = b.dataset.mode === state.mode;
    b.setAttribute('aria-checked', on ? 'true' : 'false');
    b.classList.toggle('is-active', on);
  });
  const single = el('singleBlock');
  const multi = el('multiBlock');
  if (single) single.hidden = false;
  if (multi) multi.hidden = false;
  const swarm = el('swarmPos');
  if (swarm) swarm.hidden = state.mode !== 'multi';
  updateExploreUI();
  updateRlUI();
}

function isVfhPlanner() {
  return state.mode === 'single' && samePlanner(state.planner, 'vfh');
}

function isSacPlanner() {
  return state.mode === 'single' && samePlanner(state.planner, 'sac');
}

function isTrainablePlanner() {
  return isVfhPlanner() || isSacPlanner();
}

function isFuelPlanner() {
  return state.mode === 'single' && state.planner === 'fuel_explore';
}

function updateRlUI() {
  const block = el('rlBlock');
  const singleSlot = el('rlTrainSlot');
  if (!block || !singleSlot) return;

  // Training docks under the planner when Path G (PPO) or Path H (SAC) is selected.
  const on = isTrainablePlanner() && state.page === 'single';
  if (block.parentElement !== singleSlot) singleSlot.appendChild(block);
  singleSlot.hidden = !on;
  block.hidden = !on;
  block.classList.toggle('frost-panel', on);
  block.classList.toggle('train-panel', on);
  const hint = block.querySelector('[data-i18n-html="rlTrainHint"], [data-train-hint]');
  if (hint) {
    hint.setAttribute('data-train-hint', '1');
    hint.innerHTML = isSacPlanner() ? t('sacTrainHint') : t('rlTrainHint');
  }
}

function fmtPct(v) {
  if (v == null || Number.isNaN(v)) return '—';
  return `${(Number(v) * 100).toFixed(1)}%`;
}

function applyRlTrain(rl) {
  if (!rl) return;
  const running = !!rl.running;
  const best = rl.best_success_rate ?? rl.best_rate ?? rl.best_success;
  const rate = rl.success_rate ?? rl.last_rate;
  const target = rl.target ?? 0.95;
  const reached = !running && Number(best) >= Number(target);

  const btnTrain = el('btnRlTrain');
  const btnStop = el('btnRlStop');
  if (btnTrain) btnTrain.disabled = running;
  if (btnStop) btnStop.disabled = !running;

  const st = el('rlTrainState');
  if (st) {
    st.textContent = running ? t('rlRunning') : (reached ? t('rlDone') : t('rlIdle'));
  }
  const succ = el('rlSuccess');
  if (succ) succ.textContent = fmtPct(rate);
  const bestEl = el('rlBest');
  if (bestEl) bestEl.textContent = fmtPct(best);
  const tgt = el('rlTarget');
  if (tgt) tgt.textContent = fmtPct(target);
  const steps = el('rlSteps');
  if (steps) {
    const cur = rl.timesteps ?? rl.steps;
    const total = rl.total_steps;
    if (cur != null && total != null) steps.textContent = `${cur} / ${total}`;
    else if (cur != null) steps.textContent = String(cur);
    else steps.textContent = '—';
  }
  const ck = el('rlCheckpoint');
  if (ck) {
    const fallbackName = isSacPlanner() ? 'sac_polar_local_best.pt' : 'sb3_ppo_local.zip';
    const name = rl.checkpoint ? rl.checkpoint.split('/').pop() : fallbackName;
    ck.textContent = `${name} (${rl.checkpoint_exists ? t('rlYes') : t('rlNo')})`;
  }
  const prog = el('rlProgress');
  const fill = el('rlProgressFill');
  const lab = el('rlProgressLabel');
  const showRate = Number(best ?? rate);
  if (prog && fill && lab) {
    const has = Number.isFinite(showRate);
    prog.hidden = !has && !running;
    if (has) {
      const pct = Math.max(0, Math.min(100, (showRate / Number(target || 1)) * 100));
      fill.style.width = `${pct}%`;
      lab.textContent = `${fmtPct(showRate)} / ${fmtPct(target)}`;
    } else {
      fill.style.width = '0%';
      lab.textContent = '—';
    }
  }
  const log = el('rlTrainLog');
  if (log && Array.isArray(rl.logs) && rl.logs.length) {
    log.hidden = false;
    log.textContent = rl.logs.join('\n');
    log.scrollTop = log.scrollHeight;
  }
}

const DEFAULT_ACC_SCENARIOS = [
  { id: 1, name_zh: '悬停', name_en: 'Hover', launch: 'hover.launch.py' },
  { id: 2, name_zh: '单目标点', name_en: 'Single goal', launch: 'single_goal.launch.py' },
  { id: 3, name_zh: '多目标点', name_en: 'Multi waypoint', launch: 'multi_goal.launch.py' },
  { id: 4, name_zh: '静态避障', name_en: 'Avoidance', launch: 'avoidance.launch.py' },
  { id: 5, name_zh: '狭窄通道', name_en: 'Narrow passage', launch: 'narrow_passage.launch.py' },
  { id: 6, name_zh: '稳定性展示', name_en: 'Stability', launch: 'stability_demo.launch.py' },
];

function applyAcceptance(acc) {
  const list = el('accScenarioList');
  const scenarios = (acc && acc.scenarios && acc.scenarios.length)
    ? acc.scenarios
    : DEFAULT_ACC_SCENARIOS;
  const running = !!(acc && acc.running);
  const summaryResults = ((acc && acc.summary && acc.summary.results) || []);
  const byId = {};
  summaryResults.forEach((r) => { byId[r.id] = r; });

  if (list && (!list.dataset.built || list.dataset.lang !== state.lang)) {
    list.innerHTML = '';
    scenarios.forEach((sc) => {
      const li = document.createElement('li');
      li.className = 'acc-item';
      li.dataset.id = String(sc.id);
      const name = state.lang === 'zh' ? sc.name_zh : sc.name_en;
      li.innerHTML = `
        <div class="acc-item__main">
          <span class="acc-item__id">#${sc.id}</span>
          <span class="acc-item__name">${name}</span>
          <span class="acc-item__launch mono">${sc.launch}</span>
          <span class="acc-item__result muted" data-acc-result>—</span>
        </div>
        <button type="button" class="btn btn--plain btn--tiny" data-acc-run="${sc.id}">${t('accRunOne')}</button>`;
      list.appendChild(li);
    });
    list.dataset.built = '1';
    list.dataset.lang = state.lang;
    list.querySelectorAll('[data-acc-run]').forEach((btn) => {
      btn.addEventListener('click', () => {
        startAcceptance('single', btn.getAttribute('data-acc-run')).catch((e) => alert(e.message));
      });
    });
  }

  list?.querySelectorAll('.acc-item').forEach((li) => {
    const id = Number(li.dataset.id);
    const r = byId[id];
    const badge = li.querySelector('[data-acc-result]');
    const btn = li.querySelector('[data-acc-run]');
    if (badge) {
      if (!r) badge.textContent = '—';
      else if (r.pass) badge.textContent = t('accPass');
      else badge.textContent = t('accFail');
      badge.classList.toggle('is-ok', !!r?.pass);
      badge.classList.toggle('is-bad', r != null && !r.pass);
    }
    if (btn) btn.disabled = running;
  });

  const btnAll = el('btnAccAll');
  const btnStop = el('btnAccStop');
  const rvizBox = el('accUseRviz');
  if (btnAll) btnAll.disabled = running;
  if (btnStop) btnStop.disabled = !running;
  if (rvizBox) rvizBox.disabled = running;

  const st = el('accState');
  if (st) {
    let text = running ? t('accRunning') : t('accIdle');
    if (running && acc?.config?.only) text += ` #${acc.config.only}`;
    if (running && acc?.config?.use_rviz) text += ' · RViz';
    else if (running) text += ' · no RViz';
    st.textContent = text;
  }
  const score = el('accScore');
  if (score) {
    if (acc?.summary?.passed != null && acc?.summary?.total != null) {
      score.textContent = `${acc.summary.passed}/${acc.summary.total}`;
    } else {
      score.textContent = '—';
    }
  }
  const accProg = el('accProgress');
  const accFill = el('accProgressFill');
  const accLab = el('accProgressLabel');
  if (accProg && accFill && accLab) {
    const passed = acc?.summary?.passed;
    const total = acc?.summary?.total;
    const has = passed != null && total != null && Number(total) > 0;
    accProg.hidden = !has && !running;
    if (has) {
      const pct = Math.max(0, Math.min(100, (Number(passed) / Number(total)) * 100));
      accFill.style.width = `${pct}%`;
      accLab.textContent = `${passed}/${total}`;
    } else if (running) {
      accFill.style.width = '12%';
      accLab.textContent = '…';
    } else {
      accFill.style.width = '0%';
      accLab.textContent = '—';
    }
  }
  const log = el('accLog');
  if (log && Array.isArray(acc?.logs) && acc.logs.length) {
    log.hidden = false;
    log.textContent = acc.logs.join('\n');
    log.scrollTop = log.scrollHeight;
  }
  let hint = el('accRvizHint');
  if (!hint && list?.parentElement) {
    hint = document.createElement('p');
    hint.id = 'accRvizHint';
    hint.className = 'muted tiny';
    list.parentElement.insertBefore(hint, list.nextSibling);
  }
  if (hint) {
    hint.hidden = !(acc?.config?.use_rviz && running);
    hint.textContent = t('accRvizHint');
  }
  const runHint = el('accRunHint');
  if (runHint) {
    if (running) {
      runHint.hidden = false;
      runHint.textContent = acc?.config?.use_rviz ? t('accBusyHint') : `${t('accBusyHint')} ${t('accNoRvizHint')}`;
    } else if (acc?.config && acc.config.use_rviz === false && acc.config.only) {
      runHint.hidden = false;
      runHint.textContent = t('accNoRvizHint');
    } else {
      runHint.hidden = true;
      runHint.textContent = '';
    }
  }
}

async function startAcceptance(mode, onlyId) {
  const useRviz = !!(el('accUseRviz') && el('accUseRviz').checked);
  const body = mode === 'single'
    ? { mode: 'single', only: String(onlyId), use_rviz: useRviz }
    : { mode: 'all', only: '', use_rviz: useRviz };
  const data = await api('/api/acceptance/start', {
    method: 'POST',
    body: JSON.stringify(body),
  });
  if (!data.ok) alert(data.error || t('startFailed'));
  await poll();
  pollReports().catch(() => {});
}

async function stopAcceptance() {
  await api('/api/acceptance/stop', { method: 'POST', body: '{}' });
  await poll();
  pollReports().catch(() => {});
}

function benchmarkDuration(id) {
  const value = Number(el(id)?.value);
  return Number.isFinite(value) ? Math.max(10, Math.min(600, value)) : 90;
}

function updateBenchmarkCommand() {
  const planner = el('benchmarkPlanner')?.value || 'homemade';
  const map = el('benchmarkMap')?.value || 'official_forest';
  const duration = benchmarkDuration('benchSingleDuration');
  const command = el('benchmarkCommand');
  if (command) {
    command.textContent = `python3 scripts/run_planner_benchmark.py --mode single --planner ${planner} --map ${map} --duration ${duration}`;
  }
}

function applyBenchmark(benchmark) {
  const running = !!benchmark?.running;
  const current = Number(benchmark?.current || 0);
  const total = Number(benchmark?.total || (benchmark?.config?.mode === 'single' ? 1 : 14));
  const currentCase = benchmark?.current_case || '';

  const allButton = el('btnBenchmarkAll');
  const oneButton = el('btnBenchmarkSingle');
  const stopButton = el('btnBenchmarkStop');
  if (allButton) allButton.disabled = running;
  if (oneButton) oneButton.disabled = running;
  if (stopButton) stopButton.disabled = !running;
  ['benchmarkPlanner', 'benchmarkMap', 'benchBatchDuration', 'benchSingleDuration'].forEach((id) => {
    if (el(id)) el(id).disabled = running;
  });
  if (el('btnStart')) el('btnStart').disabled = running || state.running;
  if (el('btnRestart')) el('btnRestart').disabled = running;

  const stateEl = el('benchmarkState');
  if (stateEl) {
    const base = running ? t('benchRunning') : t('benchIdle');
    stateEl.textContent = currentCase && running ? `${base} ${currentCase}` : base;
  }

  const progress = el('benchmarkProgress');
  const fill = el('benchmarkProgressFill');
  const label = el('benchmarkProgressLabel');
  if (progress && fill && label) {
    const hasProgress = total > 0 && (running || current > 0);
    progress.hidden = !hasProgress;
    const percent = total > 0 ? Math.max(0, Math.min(100, current / total * 100)) : 0;
    fill.style.width = `${percent}%`;
    label.textContent = `${current}/${total}${currentCase ? ` · ${currentCase}` : ''}`;
  }

  const summary = benchmark?.summary || {};
  const latest = el('benchmarkLatest');
  if (latest) {
    if (summary.completed_cases != null) {
      const score = summary.mean_score == null ? '—' : Number(summary.mean_score).toFixed(1);
      latest.textContent = `${summary.completed_cases}/${summary.matrix_size || 14} · success ${summary.successes || 0} · score ${score}`;
    } else {
      latest.textContent = '—';
    }
  }

  const log = el('benchmarkLog');
  if (log && Array.isArray(benchmark?.logs) && benchmark.logs.length) {
    log.hidden = false;
    log.textContent = benchmark.logs.join('\n');
    log.scrollTop = log.scrollHeight;
  }
}

async function startBenchmark(mode) {
  const single = mode === 'single';
  const body = {
    mode: single ? 'single' : 'all',
    duration: benchmarkDuration(single ? 'benchSingleDuration' : 'benchBatchDuration'),
  };
  if (single) {
    body.planner = el('benchmarkPlanner').value;
    body.map = el('benchmarkMap').value;
  }
  const data = await api('/api/benchmark/start', {
    method: 'POST',
    body: JSON.stringify(body),
  });
  if (!data.ok) throw new Error(data.error || t('startFailed'));
  await poll();
}

async function stopBenchmark() {
  await api('/api/benchmark/stop', { method: 'POST', body: '{}' });
  await poll();
  pollReports().catch(() => {});
}

function updateExploreUI() {
  const block = el('exploreBlock');
  const hint = el('goalHintDefault');
  if (block) block.hidden = !isFuelPlanner();
  if (hint) hint.hidden = isFuelPlanner();
  const btn = el('btnExplore');
  if (btn) btn.disabled = !(state.running && isFuelPlanner());
}

function plannerLabel(meta) {
  if (meta.label) return meta.label;
  const keys = PLANNER_I18N[meta.id || meta];
  if (keys) return t(keys.label);
  return meta.id || String(meta);
}

function plannerDesc(meta) {
  if (meta.desc) return meta.desc;
  if (meta.principle) return meta.principle;
  const keys = PLANNER_I18N[meta.id];
  if (keys) return t(keys.desc);
  return '';
}

function renderPlannersForLang() {
  const reg = state.lang === 'zh' && state.plannerRegistryZh.length
    ? state.plannerRegistryZh
    : (state.plannerRegistryEn.length ? state.plannerRegistryEn : state.plannerRegistry);
  if (reg.length) renderPlanners(reg);
  else if (Object.keys(state.planners).length) renderPlannersLegacy(state.planners);
}

function pathLetterFromLabel(label, fallback) {
  const m = String(label || '').match(/(?:Path|路径)\s*([A-H])/i);
  if (m) return m[1].toUpperCase();
  const id = String(fallback || '');
  const map = {
    homemade: 'A', ego: 'B', gcopter: 'C', fuel_explore: 'D',
    mighty: 'E', fast_planner: 'F', rl: 'G', vfh: 'G', sac: 'H',
  };
  return map[id] || (id ? id.slice(0, 1).toUpperCase() : '·');
}

function renderPlanners(registry) {
  state.plannerRegistry = Array.isArray(registry) ? registry : [];
  // Nest CSS option cards inside #plannerShell (one LiquidGlass panel),
  // matching #mapPanel → #mapGrid.
  const root = el('plannerGroups');
  clearHoistedOptionCards();
  if (root) root.innerHTML = '';

  const byClass = {};
  for (const p of state.plannerRegistry) {
    if (p.class === 'multi') continue;
    (byClass[p.class] ||= []).push(p);
  }

  if (!root) {
    selectPlanner(state.planner, false);
    refreshLiquidCards();
    return;
  }

  for (const cls of PLANNER_CLASS_ORDER) {
    const items = byClass[cls];
    if (!items || !items.length) continue;
    const title = document.createElement('h3');
    title.className = 'planner-group__title';
    title.textContent = t(CLASS_I18N[cls] || cls);
    root.appendChild(title);
    for (const p of items) {
      const pid = p.id;
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'planner-card';
      btn.dataset.planner = pid;
      const label = plannerLabel(p);
      const mark = pathLetterFromLabel(label, pid);
      btn.innerHTML = `
        <span class="choice-row__id">${mark}</span>
        <span class="choice-row__body">
          <span class="title">${label}</span>
          <span class="desc">${plannerDesc(p)}</span>
        </span>`;
      btn.addEventListener('click', () => selectPlanner(pid));
      root.appendChild(btn);
    }
  }
  selectPlanner(state.planner, false);
  refreshLiquidCards();
}

function renderPlannersLegacy(planners) {
  state.planners = planners || {};
  const root = el('plannerGroups');
  clearHoistedOptionCards();
  if (root) root.innerHTML = '';
  if (!root) {
    selectPlanner(state.planner, false);
    refreshLiquidCards();
    return;
  }
  for (const key of ['homemade', 'ego', 'gcopter', 'fuel_explore', 'mighty', 'fast_planner', 'rl', 'sac']) {
    if (!PLANNER_I18N[key]) continue;
    const keys = PLANNER_I18N[key];
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'planner-card';
    btn.dataset.planner = key;
    const label = t(keys.label);
    btn.innerHTML = `
      <span class="choice-row__id">${pathLetterFromLabel(label, key)}</span>
      <span class="choice-row__body">
        <span class="title">${label}</span>
        <span class="desc">${t(keys.desc)}</span>
      </span>`;
    btn.addEventListener('click', () => selectPlanner(key));
    root.appendChild(btn);
  }
  selectPlanner(state.planner, false);
  refreshLiquidCards();
}

function renderMulti(modes) {
  state.multiModes = modes || {};
  const grid = el('multiGrid');
  clearHoistedOptionCards();
  if (grid) grid.innerHTML = '';
  if (!grid) {
    selectMulti(state.multiMode, false);
    refreshLiquidCards();
    return;
  }
  let i = 0;
  for (const key of MULTI_ORDER) {
    const meta = modes[key];
    if (!meta) continue;
    i += 1;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'planner-card';
    btn.dataset.multi = key;
    const label = state.lang === 'zh' ? meta.label_zh : meta.label_en;
    const desc = state.lang === 'zh' ? meta.desc_zh : meta.desc_en;
    btn.innerHTML = `
      <span class="choice-row__id">${i}</span>
      <span class="choice-row__body">
        <span class="title">${label}</span>
        <span class="desc">${desc || ''}</span>
      </span>`;
    btn.addEventListener('click', () => selectMulti(key));
    grid.appendChild(btn);
  }
  selectMulti(state.multiMode, false);
  refreshLiquidCards();
}

function difficultyBadge(diff) {
  if (!diff) return '';
  const key = DIFF_I18N[diff] || diff;
  return `<span class="diff-badge diff-badge--${diff}">${t(key)}</span>`;
}

function buildMapCardNodes(maps) {
  const nodes = [];
  let i = 0;
  for (const key of MAP_ORDER) {
    const meta = maps[key];
    if (!meta) continue;
    i += 1;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'map-card';
    btn.dataset.map = key;
    const label = state.lang === 'zh' ? meta.label_zh : meta.label_en;
    const desc = state.lang === 'zh' ? meta.desc_zh : meta.desc_en;
    const diff = difficultyBadge(meta.difficulty);
    const seedHint = meta.seed != null ? `<span class="map-seed">seed ${meta.seed}</span>` : '';
    btn.innerHTML = `
      <span class="choice-row__id">${i}</span>
      <span class="choice-row__body">
        <span class="title">${label}</span>
        <span class="desc">${desc || ''}</span>
        <span class="map-card__meta">${diff}${seedHint}</span>
      </span>`;
    btn.addEventListener('click', () => selectMap(key));
    nodes.push(btn);
  }
  return nodes;
}

function placeMapCards(lg) {
  const panel = el('mapPanel');
  const grid = el('mapGrid');
  // Always clear any cards that were wrongly promoted onto the mission grid
  // (dense packing used to pull map #1 up beside multi/planner options).
  if (lg) lg.querySelectorAll(':scope > .map-card').forEach((n) => n.remove());
  document.querySelectorAll('#missionLgSingle > .map-card, #missionLgMulti > .map-card')
    .forEach((n) => n.remove());
  const nodes = buildMapCardNodes(state.maps || {});
  if (!nodes.length || !grid) return false;
  grid.className = 'choice-list choice-list--maps';
  grid.innerHTML = '';
  for (const n of nodes) grid.appendChild(n);
  // Keep mapPanel under the active mission root when mounted.
  if (lg && panel && panel.parentElement !== lg && !panel.hidden) {
    /* mountMapPanel owns moving the shell */
  }
  return true;
}

function renderMaps(maps) {
  state.maps = maps || {};
  const panel = el('mapPanel');
  let lg = panel && panel.parentElement && panel.parentElement.classList.contains('mission-lg')
    ? panel.parentElement
    : null;
  if (!lg) {
    const page = document.documentElement.getAttribute('data-page') || state.page;
    lg = page === 'multi' ? el('missionLgMulti') : el('missionLgSingle');
  }
  if (!placeMapCards(lg)) {
    const grid = el('mapGrid');
    if (grid) {
      grid.className = 'choice-list choice-list--maps';
      grid.innerHTML = '';
      for (const n of buildMapCardNodes(state.maps)) grid.appendChild(n);
    }
  }
  if (state.map && state.maps[state.map]) selectMap(state.map, false);
  updateMultiMapLock();
  updateMultiFieldVisibility();
  refreshLiquidCards();
}

function defaultMapForPlanner(plannerId) {
  const key = normPlanner(plannerId);
  const defaults = state.mapDefaults || {};
  return defaults[key] || defaults[plannerId] || null;
}

function highlightPlanner(key) {
  state.planner = key;
  document.querySelectorAll('[data-planner]').forEach((card) => {
    const on = samePlanner(card.dataset.planner, key);
    card.setAttribute('aria-checked', on ? 'true' : 'false');
    card.classList.toggle('is-active', on);
  });
}

function selectPlanner(key, sync = true) {
  highlightPlanner(key);
  // Do not lock / bounce the map to DEFAULT_MAP_BY_PLANNER.
  // Architecture already supports any planner × any map via map_stack + cloud_bridge.
  // Only fill a default when nothing valid is selected yet.
  if (state.mode !== 'multi' && (!state.map || !state.maps[state.map])) {
    const defMap = defaultMapForPlanner(key);
    if (defMap && state.maps[defMap]) {
      selectMap(defMap, false);
    }
  }
  if ((samePlanner(key, 'vfh') || samePlanner(key, 'sac')) && el('maxVel')) {
    const cur = Number(el('maxVel').value);
    if (!Number.isFinite(cur) || cur > 1.1 || cur < 0.5) {
      el('maxVel').value = 0.85;
    }
  }
  updateExploreUI();
  updateRlUI();
  if (sync) syncConfig();
}

function selectMulti(key, sync = true) {
  state.multiMode = key;
  document.querySelectorAll('[data-multi]').forEach((card) => {
    card.setAttribute('aria-checked', card.dataset.multi === key ? 'true' : 'false');
    card.classList.toggle('is-active', card.dataset.multi === key);
  });
  // shared_field / formation always run dense_field (launch hard-lock).
  // ego_swarm keeps free map choice; seed default forest only if empty.
  if (state.mode === 'multi' && (key === 'shared_field' || key === 'formation')) {
    selectMap('dense_field', false);
    if (key === 'shared_field' && el('numDrones')) el('numDrones').value = 2;
    if (key === 'formation' && el('numDrones')) el('numDrones').value = 3;
  } else if (state.mode === 'multi' && key === 'ego_swarm' && !state.map) {
    state.map = 'official_forest';
    selectMap(state.map, false);
  }
  updateMultiMapLock();
  updateMultiFieldVisibility();
  if (sync) syncConfig();
}

/** Homemade multi modes only support dense_field — grey out other map cards. */
function isDenseOnlyMultiMode() {
  // The visible page is authoritative. A stale server/mode response must never
  // carry the multi-drone dense-map restriction onto the single-flight page.
  return state.page === 'multi'
    && (state.multiMode === 'shared_field' || state.multiMode === 'formation');
}

function updateMultiMapLock() {
  const lock = isDenseOnlyMultiMode();
  document.querySelectorAll('.map-card').forEach((card) => {
    const allowed = !lock || card.dataset.map === 'dense_field';
    card.disabled = !allowed;
    card.classList.toggle('is-locked-out', !allowed);
    card.title = allowed ? '' : (state.lang === 'zh'
      ? '共享空域 / 编队仅支持密集场景'
      : 'Shared field / formation only support dense_field');
  });
}

function updateMultiFieldVisibility() {
  const numLabel = el('numDrones')?.closest('label');
  const formLabel = el('formation')?.closest('label');
  const formSelect = el('formation');
  const isEgo = state.multiMode === 'ego_swarm';
  const isForm = state.multiMode === 'formation';
  if (numLabel) numLabel.hidden = state.mode === 'multi' && !isEgo;
  // Formation only applies to formation mode — keep visible but greyed out otherwise.
  if (formLabel) {
    formLabel.hidden = false;
    const locked = state.mode === 'multi' && !isForm;
    formLabel.classList.toggle('is-disabled', locked);
    if (formSelect) {
      formSelect.disabled = locked;
      formSelect.title = locked
        ? (state.lang === 'zh' ? '队形仅在编队模式下可选' : 'Formation is only available in Formation mode')
        : '';
    }
  }
}

function selectMap(key, sync = true) {
  const lock = isDenseOnlyMultiMode();
  if (lock && key !== 'dense_field') {
    key = 'dense_field';
  }
  state.map = key;
  document.querySelectorAll('.map-card').forEach((card) => {
    const on = card.dataset.map === key;
    card.setAttribute('aria-checked', on ? 'true' : 'false');
    card.classList.toggle('is-active', on);
  });
  const meta = state.maps[key];
  if (meta && meta.seed != null && document.activeElement?.id !== 'seed') {
    el('seed').value = meta.seed;
  }
  updateMultiMapLock();
  if (sync) syncConfig();
}

function selectRunMode(mode, sync = true) {
  state.mode = mode;
  updateModeUI();
  if (mode === 'multi') {
    state.multiMode = state.multiMode || 'ego_swarm';
    state.map = state.map || 'official_forest';
    selectMulti(state.multiMode, false);
  } else {
    updateMultiMapLock();
    updateMultiFieldVisibility();
  }
  if (sync) syncConfig();
}

function mapGoalDefaults() {
  const meta = state.maps[state.map] || {};
  return {
    goal_x: meta.goal_x ?? 15.0,
    goal_y: meta.goal_y ?? 0.0,
    goal_z: meta.goal_z ?? 1.0,
  };
}

function formConfig() {
  const goals = mapGoalDefaults();
  const planner = samePlanner(state.planner, 'vfh') ? 'rl' : state.planner;
  // Page is source of truth: Multi page must never start a single-drone launch
  // even if a stale poll briefly reset state.mode to "single".
  const mode = state.page === 'multi' ? 'multi'
    : state.page === 'single' ? 'single'
    : state.mode;
  if (mode === 'multi') state.mode = 'multi';
  if (mode === 'single') state.mode = 'single';
  const cfg = {
    mode,
    planner,
    map: state.map,
    seed: Number(el('seed').value) || 1,
    max_vel: Number(el('maxVel').value) || 1.2,
    use_rviz: el('useRviz').checked,
    goal_x: goals.goal_x,
    goal_y: goals.goal_y,
    goal_z: goals.goal_z,
  };
  // Only attach multi fields when launching multi — avoids server mistaking
  // a leftover multi_mode=ego_swarm for an EGO-Swarm start on Path G.
  if (mode === 'multi') {
    cfg.multi_mode = state.multiMode;
    cfg.num_drones = Number(el('numDrones').value) || 2;
    cfg.formation = el('formation').value;
  }
  return cfg;
}

async function syncConfig() {
  const data = await api('/api/config', { method: 'POST', body: JSON.stringify(formConfig()) });
  el('cmdPreview').textContent = data.cmd || '—';
}

function placeMissionRunBar(page) {
  // Buttons live under the map panel; visibility follows xyTrackPanel.
  const bar = el('missionRunBar');
  const track = el('xyTrackPanel');
  if (!bar || !track) return;
  if (bar.parentElement !== track) track.appendChild(bar);
  bar.hidden = !(page === 'single' || page === 'multi') || track.hidden;
}

function placeMonitorDock(page) {
  const dock = el('monitorDock');
  const track = el('xyTrackPanel');
  const labSlot = el('monitorLabSlot');
  if (!dock) return;

  if (page === 'lab' && labSlot) {
    if (dock.parentElement !== labSlot) labSlot.appendChild(dock);
    labSlot.hidden = false;
    dock.hidden = false;
    dock.classList.add('monitor-dock--lab');
    return;
  }

  if (labSlot) labSlot.hidden = true;
  dock.classList.remove('monitor-dock--lab');
  if (track && (page === 'single' || page === 'multi')) {
    if (dock.parentElement !== track) track.appendChild(dock);
    dock.hidden = !!track.hidden;
  } else {
    dock.hidden = true;
  }
}

function setRunningUI(running) {
  state.running = !!running;
  const pill = el('runPill');
  if (pill) pill.dataset.state = running ? 'running' : 'idle';
  const label = el('runLabel');
  if (label) label.textContent = running ? t('running') : t('idle');
  const btnStart = el('btnStart');
  const btnStop = el('btnStop');
  if (btnStart) btnStart.disabled = running;
  if (btnStop) btnStop.disabled = !running;
  updateExploreUI();
}

function fmt(n, digits = 2) {
  if (n === null || n === undefined || Number.isNaN(n)) return '—';
  return Number(n).toFixed(digits);
}

function collectWarnings(st) {
  const warnings = [];
  if (state.offline) warnings.push(t('warnOffline'));
  if (st.fallback_active) warnings.push(t('warnFallback'));
  const ps = st.planner_status;
  if (ps && String(ps.state).toUpperCase() === 'FAIL') warnings.push(t('warnPlannerFail'));
  const pd = st.planner_diagnostics;
  if (pd && pd.fallback_active && !st.fallback_active) warnings.push(t('warnFallback'));
  if (Array.isArray(st.logs)) {
    const tail = st.logs.slice(-6).join('\n').toLowerCase();
    if (tail.includes('error') || tail.includes('[warn')) {
      warnings.push(state.lang === 'zh' ? '日志含警告' : 'Log warnings');
    }
  }
  return warnings;
}

function applyDiagnostics(st) {
  const ps = st.planner_status;
  const pd = st.planner_diagnostics;
  const fallback = st.fallback_active || (pd && pd.fallback_active);

  const stateEl = el('diagState');
  if (stateEl) {
    stateEl.textContent = (pd && pd.state) || (ps && ps.state) || '—';
  }
  const fb = el('diagFallback');
  if (fb) {
    fb.textContent = fallback ? t('diagActive') : t('diagInactive');
    fb.classList.toggle('is-warn', !!fallback);
  }
  const reason = el('diagReason');
  if (reason) {
    reason.textContent = (pd && pd.fallback_reason) || (ps && ps.message) || '—';
  }
  const clearance = el('diagClearance');
  if (clearance) {
    const c = pd && pd.clearance_m != null ? pd.clearance_m : (ps && ps.min_obstacle_distance);
    clearance.textContent = c != null ? `${fmt(c, 2)} m` : '—';
  }
  const solve = el('diagSolve');
  if (solve) {
    solve.textContent = pd && pd.solve_time_ms != null ? `${fmt(pd.solve_time_ms, 1)} ms` : '—';
  }
}

function renderReports(files) {
  const list = el('reportList');
  if (!list) return;
  list.innerHTML = '';
  if (!files || !files.length) {
    const li = document.createElement('li');
    li.className = 'muted';
    li.textContent = t('reportsEmpty');
    list.appendChild(li);
    return;
  }
  for (const f of files) {
    const li = document.createElement('li');
    const when = new Date(f.mtime * 1000).toLocaleString(state.lang === 'zh' ? 'zh-CN' : 'en');
    li.innerHTML = `
      <span class="report-kind report-kind--${f.kind}">${f.kind}</span>
      <span class="report-name">${f.name}</span>
      <span class="report-meta muted">${when}</span>`;
    list.appendChild(li);
  }
}

async function pollReports() {
  try {
    const data = await api('/api/reports');
    state.reports = data.files || [];
    const dir = el('reportDir');
    if (dir && data.report_dir) {
      const rel = data.report_dir.includes('report') ? 'report/' : data.report_dir;
      dir.textContent = rel.endsWith('/') ? rel : `${rel}/`;
    }
    renderReports(state.reports);
  } catch {
    const list = el('reportList');
    if (list && !state.reports.length) {
      list.innerHTML = `<li class="muted">${t('reportsError')}</li>`;
    }
  }
}

function applyStatus(st) {
  state.offline = false;

  if (st.planner_registry) {
    state.plannerRegistryEn = st.planner_registry;
    if (st.planner_registry_zh) state.plannerRegistryZh = st.planner_registry_zh;
    const reg = state.lang === 'zh' && state.plannerRegistryZh.length
      ? state.plannerRegistryZh
      : state.plannerRegistryEn;
    const regKey = `${state.lang}:${JSON.stringify(reg)}`;
    if (regKey !== state._registryKey) {
      state._registryKey = regKey;
      state.plannerRegistry = reg;
      renderPlanners(reg);
    }
  } else if (st.planners && !state.plannerRegistry.length) {
    renderPlannersLegacy(st.planners);
  }

  if (st.map_defaults) state.mapDefaults = st.map_defaults;
  if (st.multi_modes && !Object.keys(state.multiModes).length) renderMulti(st.multi_modes);
  if (st.maps && !Object.keys(state.maps).length) renderMaps(st.maps);

  setRunningUI(!!st.running);
  el('pid').textContent = st.pid ?? '—';
  el('uptime').textContent = st.running ? `${fmt(st.uptime_s, 0)} s` : '—';
  el('cmdPreview').textContent = st.cmd || '—';

  const o = st.odom;
  const posText = o ? `${fmt(o.x)}, ${fmt(o.y)}, ${fmt(o.z)}` : '—';
  const velText = o ? `${fmt(o.vx)}, ${fmt(o.vy)}, ${fmt(o.vz)}` : '—';
  if (el('pos')) el('pos').textContent = posText;
  if (el('vel')) el('vel').textContent = velText;
  if (el('posHome')) el('posHome').textContent = posText;
  if (el('velHome')) el('velHome').textContent = velText;

  ingestXyFromStatus(st);

  const warnings = collectWarnings(st);
  const warnText = warnings.length ? warnings.join(' · ') : t('warnNone');
  ['warnings', 'warningsHome'].forEach((id) => {
    const warnEl = el(id);
    if (!warnEl) return;
    warnEl.textContent = warnText;
    warnEl.classList.toggle('is-warn', warnings.length > 0);
  });

  const swarm = st.swarm_odom || {};
  const keys = Object.keys(swarm).sort();
  if (keys.length) {
    el('swarmPos').hidden = false;
    el('swarmPos').textContent = keys.map((k) => {
      const p = swarm[k];
      return `${k}: ${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}`;
    }).join('\n');
  }

  const ex = el('exploreStatus');
  if (ex) {
    ex.textContent = st.exploration_status || '—';
  }

  applyDiagnostics(st);
  applyRlTrain(isSacPlanner() ? st.sac_train : st.rl_train);
  applyAcceptance(st.acceptance);
  applyBenchmark(st.benchmark);

  if (!state.clearLocalLogs && Array.isArray(st.logs)) {
    el('logView').textContent = st.logs.join('\n');
    el('logView').scrollTop = el('logView').scrollHeight;
  }

  if (st.config && !st.running && document.activeElement?.tagName !== 'INPUT') {
    const c = st.config;
    el('seed').value = c.seed ?? 1;
    el('maxVel').value = c.max_vel ?? 1.2;
    el('useRviz').checked = !!c.use_rviz;
    el('numDrones').value = c.num_drones ?? 2;
    el('formation').value = ['line', 'column', 'v'].includes(c.formation)
      ? c.formation
      : 'v';
    // Never let a stale server "single" wipe Multi-page selection (that used to
    // start planner_sim with 1 drone while the UI still showed 共享空域避障).
    if (state.page === 'multi') {
      state.mode = 'multi';
      if (c.multi_mode && !document.querySelector('[data-multi].is-active')) {
        state.multiMode = c.multi_mode;
      }
    } else if (state.page === 'single') {
      state.mode = 'single';
      if (c.planner) state.planner = c.planner;
    } else {
      if (c.mode) state.mode = c.mode;
      if (c.planner) state.planner = c.planner;
      if (c.multi_mode) state.multiMode = c.multi_mode;
    }
    if (c.map && !(state.page === 'multi'
        && (state.multiMode === 'shared_field' || state.multiMode === 'formation'))) {
      state.map = c.map;
    }
    updateModeUI();
    // Highlight only — never call selectPlanner here (it used to stomp the map every poll).
    if (state.mode === 'multi') {
      selectMulti(state.multiMode, false);
    } else {
      highlightPlanner(state.planner);
      updateExploreUI();
      updateRlUI();
    }
    selectMap(state.map, false);
  }
}

async function poll() {
  try {
    applyStatus(await api('/api/status'));
  } catch (err) {
    state.offline = true;
    el('runLabel').textContent = t('offline');
    el('runPill').dataset.state = 'idle';
    const warnEl = el('warnings');
    if (warnEl) {
      warnEl.textContent = t('warnOffline');
      warnEl.classList.add('is-warn');
    }
  }
}

async function startSim() {
  state.clearLocalLogs = false;
  clearXyTrack();
  xyTrack.occupancy = null;
  const cfg = formConfig();
  const data = await api('/api/start', { method: 'POST', body: JSON.stringify(cfg) });
  if (!data.ok) {
    alert(data.error || t('startFailed'));
  } else if (cfg.mode === 'single' && data.cmd
      && /(shared_field|ego_swarm|formation)\.launch\.py/.test(data.cmd)) {
    alert(`Single start launched a multi launch:\n${data.cmd}\n\nExpected planner_sim.launch.py.`);
  } else if (cfg.mode === 'multi' && data.cmd
      && !/(shared_field|ego_swarm|formation)\.launch\.py/.test(data.cmd)) {
    alert(`Multi start launched the wrong file:\n${data.cmd}\n\nExpected shared_field / ego_swarm / formation.`);
  }
  if (el('cmdPreview') && data.cmd) el('cmdPreview').textContent = data.cmd;
  await poll();
  setTimeout(pollOccupancy, 1500);
}

async function stopSim() {
  await api('/api/stop', { method: 'POST', body: '{}' });
  await poll();
}

async function restartSim() {
  state.clearLocalLogs = false;
  clearXyTrack();
  xyTrack.occupancy = null;
  await api('/api/restart', { method: 'POST', body: JSON.stringify(formConfig()) });
  await poll();
  setTimeout(pollOccupancy, 1500);
}

async function startRlTrain() {
  if (isSacPlanner()) {
    const data = await api('/api/sac/train', {
      method: 'POST',
      body: JSON.stringify({
        target: 0.90,
        steps: 5000000,
        dense_heavy: true,
        eval_every: 5000,
        eval_episodes: 60,
        resume: 'auto',
        fresh: false,
        n_envs: 4,
        batch_size: 128,
        updates_per_step: 2,
        finetune_lr: 1e-4,
        persist_buffer: true,
        open_monitor: true,
      }),
    });
    if (!data.ok) alert(data.error || t('startFailed'));
    await poll();
    return;
  }
  const data = await api('/api/rl/train', {
    method: 'POST',
    body: JSON.stringify({ target: 0.95, steps: 2000000, easy: false, n_envs: 8 }),
  });
  if (!data.ok) alert(data.error || t('startFailed'));
  await poll();
}

async function stopRlTrain() {
  if (isSacPlanner()) {
    await api('/api/sac/stop', { method: 'POST', body: '{}' });
  } else {
    await api('/api/rl/stop', { method: 'POST', body: '{}' });
  }
  await poll();
}

document.querySelectorAll('.mode-chip').forEach((btn) => {
  btn.addEventListener('click', () => {
    selectRunMode(btn.dataset.mode);
    showPage(btn.dataset.mode === 'multi' ? 'multi' : 'single', { sync: false });
  });
});

document.querySelectorAll('[data-preset]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const p = btn.dataset.preset;
    if (p === 'cross') {
      selectRunMode('single', false);
      selectPlanner('gcopter', false);
      selectMap('official_forest', false);
      el('seed').value = 1;
      showPage('single', { sync: false });
    } else if (p === 'swarm') {
      selectRunMode('multi', false);
      selectMulti('ego_swarm', false);
      selectMap('official_forest', false);
      el('numDrones').value = 2;
      el('seed').value = 1;
      showPage('multi', { sync: false });
    } else if (p === 'home') {
      selectRunMode('single', false);
      selectPlanner('homemade', false);
      selectMap('dense_field', false);
      el('seed').value = 42;
      showPage('single', { sync: false });
    } else if (p === 'fuel') {
      selectRunMode('single', false);
      selectPlanner('fuel_explore', false);
      selectMap('dense_field', false);
      el('seed').value = 42;
      showPage('single', { sync: false });
    }
    syncConfig();
  });
});

async function triggerExplore() {
  const meta = state.maps[state.map] || {};
  const data = await api('/api/goal', {
    method: 'POST',
    body: JSON.stringify({
      x: meta.explore_init_x ?? meta.init_x ?? 1.0,
      y: meta.explore_init_y ?? meta.init_y ?? 5.0,
      z: meta.explore_init_z ?? meta.init_z ?? 1.5,
      yaw: 0,
    }),
  });
  if (!data.ok) alert(data.error || t('startFailed'));
  await poll();
}

document.querySelectorAll('.lang-btn').forEach((btn) => {
  btn.addEventListener('click', () => setLang(btn.dataset.lang));
});

document.querySelectorAll('.seg__btn[data-ambiance], .segment__btn[data-ambiance]').forEach((btn) => {
  btn.addEventListener('click', () => setAmbiance(btn.dataset.ambiance));
});

document.querySelectorAll('[data-copy]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const target = el(btn.dataset.copy);
    if (!target) return;
    const text = target.textContent || '';
    navigator.clipboard.writeText(text).catch(() => {});
  });
});

const btnRlTrain = el('btnRlTrain');
if (btnRlTrain) {
  btnRlTrain.addEventListener('click', () => startRlTrain().catch((e) => alert(e.message)));
}
const btnRlStop = el('btnRlStop');
if (btnRlStop) {
  btnRlStop.addEventListener('click', () => stopRlTrain().catch((e) => alert(e.message)));
}

const btnAccAll = el('btnAccAll');
if (btnAccAll) {
  btnAccAll.addEventListener('click', () => startAcceptance('all').catch((e) => alert(e.message)));
}
const btnAccStop = el('btnAccStop');
if (btnAccStop) {
  btnAccStop.addEventListener('click', () => stopAcceptance().catch((e) => alert(e.message)));
}

const btnBenchmarkAll = el('btnBenchmarkAll');
if (btnBenchmarkAll) {
  btnBenchmarkAll.addEventListener('click', () => startBenchmark('all').catch((e) => alert(e.message)));
}
const btnBenchmarkSingle = el('btnBenchmarkSingle');
if (btnBenchmarkSingle) {
  btnBenchmarkSingle.addEventListener('click', () => startBenchmark('single').catch((e) => alert(e.message)));
}
const btnBenchmarkStop = el('btnBenchmarkStop');
if (btnBenchmarkStop) {
  btnBenchmarkStop.addEventListener('click', () => stopBenchmark().catch((e) => alert(e.message)));
}
['benchmarkPlanner', 'benchmarkMap', 'benchSingleDuration'].forEach((id) => {
  if (el(id)) el(id).addEventListener('change', updateBenchmarkCommand);
});
updateBenchmarkCommand();

const btnStart = el('btnStart');
const btnStop = el('btnStop');
const btnRestart = el('btnRestart');
if (btnStart) btnStart.addEventListener('click', () => startSim().catch((e) => alert(e.message)));
if (btnStop) btnStop.addEventListener('click', () => stopSim().catch((e) => alert(e.message)));
if (btnRestart) btnRestart.addEventListener('click', () => restartSim().catch((e) => alert(e.message)));

const btnClearTrack = el('btnClearTrack');
if (btnClearTrack) {
  btnClearTrack.addEventListener('click', () => clearXyTrack());
}
const btnMapHeading = el('btnMapHeading');
if (btnMapHeading) {
  btnMapHeading.addEventListener('click', () => setMapOrientation(true));
}
const btnMapNorth = el('btnMapNorth');
if (btnMapNorth) {
  btnMapNorth.addEventListener('click', () => setMapOrientation(false));
}
try {
  const savedH = localStorage.getItem('drone_dash_map_heading');
  if (savedH === '0') setMapOrientation(false);
  else setMapOrientation(true);
} catch (e) {
  setMapOrientation(true);
}

function initPageSplitResize() {
  document.querySelectorAll('.page-split__splitter').forEach((handle) => {
    handle.addEventListener('pointerdown', (ev) => {
      const split = handle.closest('.page-split');
      if (!split) return;
      ev.preventDefault();
      const rect = split.getBoundingClientRect();
      const onMove = (e) => {
        const x = e.clientX - rect.left;
        const sidePx = Math.min(Math.max(rect.right - e.clientX, 220), rect.width * 0.7);
        const mainPx = Math.max(200, rect.width - sidePx - 16);
        split.style.gridTemplateColumns = `${mainPx}px 8px ${sidePx}px`;
        drawXyTrack();
      };
      const onUp = () => {
        document.body.classList.remove('is-resizing-split');
        window.removeEventListener('pointermove', onMove);
        window.removeEventListener('pointerup', onUp);
        drawXyTrack();
      };
      document.body.classList.add('is-resizing-split');
      window.addEventListener('pointermove', onMove);
      window.addEventListener('pointerup', onUp);
      onMove(ev);
    });
  });
}

function initMapHeightResize() {
  const handle = el('xyTrackResize');
  const stage = el('xyTrackStage');
  if (!handle || !stage) return;
  try {
    const saved = parseFloat(localStorage.getItem('drone_dash_map_h') || '');
    if (Number.isFinite(saved) && saved >= 160) {
      stage.style.setProperty('--map-h', `${saved}px`);
    }
  } catch (e) { /* ignore */ }
  handle.addEventListener('pointerdown', (ev) => {
    ev.preventDefault();
    const startY = ev.clientY;
    const startH = stage.getBoundingClientRect().height;
    const onMove = (e) => {
      const next = Math.min(Math.max(startH + (e.clientY - startY), 160), window.innerHeight * 0.72);
      stage.style.setProperty('--map-h', `${next}px`);
      drawXyTrack();
    };
    const onUp = () => {
      document.body.classList.remove('is-resizing-map');
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
      try {
        localStorage.setItem('drone_dash_map_h', String(stage.getBoundingClientRect().height));
      } catch (e) { /* ignore */ }
      drawXyTrack();
    };
    document.body.classList.add('is-resizing-map');
    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
  });
}

initPageSplitResize();
initMapHeightResize();
window.addEventListener('resize', () => drawXyTrack());

const btnExplore = el('btnExplore');
if (btnExplore) {
  btnExplore.addEventListener('click', () => triggerExplore().catch((e) => alert(e.message)));
}
el('btnClearLog').addEventListener('click', (e) => {
  e.preventDefault();
  e.stopPropagation();
  state.clearLocalLogs = true;
  el('logView').textContent = '';
  setTimeout(() => { state.clearLocalLogs = false; }, 1500);
});

['seed', 'maxVel', 'useRviz', 'numDrones', 'formation'].forEach((id) => {
  el(id).addEventListener('change', () => syncConfig().catch(() => {}));
});

el('hostHint').textContent = location.host;

document.querySelectorAll('[data-nav]').forEach((btn) => {
  btn.addEventListener('click', (e) => {
    const page = btn.dataset.nav;
    if (!page || !PAGES.includes(page)) return;
    if (btn.tagName === 'A' && btn.getAttribute('href')?.startsWith('#')) {
      e.preventDefault();
    }
    showPage(page);
  });
});

/* Guide TOC: scroll only the right column, keep left nav fixed. */
document.querySelectorAll('.guide-toc a[href^="#"]').forEach((a) => {
  a.addEventListener('click', (e) => {
    const id = (a.getAttribute('href') || '').slice(1);
    const target = id && document.getElementById(id);
    if (!target) return;
    e.preventDefault();
    target.scrollIntoView({ behavior: 'smooth', block: 'start' });
  });
});

window.addEventListener('hashchange', () => {
  let page = location.hash.replace(/^#/, '');
  if (page === 'home') page = DEFAULT_PAGE;
  if (PAGES.includes(page) && page !== state.page) showPage(page, { sync: false });
});

bindBackgroundPicker();
applyI18n();
{
  let initial = DEFAULT_PAGE;
  try {
    initial = localStorage.getItem('drone_dash_page') || initial;
  } catch (e) { /* ignore */ }
  if (initial === 'home' || !PAGES.includes(initial)) initial = DEFAULT_PAGE;
  const fromHash = location.hash.replace(/^#/, '');
  if (fromHash === 'home') initial = DEFAULT_PAGE;
  else if (PAGES.includes(fromHash)) initial = fromHash;
  showPage(initial, { sync: false });
}
drawXyTrack();
poll();
pollReports();
pollOccupancy();
pollMapLive();
setInterval(poll, 1000);
setInterval(pollReports, 15000);
setInterval(pollOccupancy, 2000);
setInterval(pollMapLive, 100);
