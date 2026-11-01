# EvergreenSLAM

常驻运行的 2D 激光 SLAM，支持命名地点，用作 LLM agent 的空间记忆。

[![ci](https://github.com/ThomasZB/EvergreenSLAM/actions/workflows/ci.yml/badge.svg)](https://github.com/ThomasZB/EvergreenSLAM/actions/workflows/ci.yml)

[English](../README.md) | 中文

---

https://github.com/user-attachments/assets/fbfdee89-250d-42ae-aad3-e9d3cb0cb1f4

<p align="center"><sub>一个 agent（Claude Code）以 EvergreenSLAM 作为家用机器人的记忆完成的两次会话，一共 1 分 44 秒，标注了当前时间和播放倍速。第一天从空地图开始：在去阳台拍照任务中，agent 只在一条推断笔记里提到过的阳台，经 <code>egs fs grep</code> 找到并拍下。第二天，机器人断电过夜后，一个没有对话历史的新 agent 会话根据记忆里的观察记录找到剪刀并拍照。全程没有遥控，通过自然语言控制，没有任何手动控制建图定位的操作。完整视频（3 分 18 秒，英文字幕）：<a href="https://youtu.be/FXoj2yCrbuo">YouTube</a>。详见<a href="#agent-探索记录回访">「实验」</a>。</sub></p>

## Hello, world

```
$ egs here
current places/dock 2.4m
closest places/dock 2.4m frozen; places/kitchen 15.3m frozen
robot 0.48,1.02,-2.72  keyframe 0s  @solve 135
base 0.75 (cover 98%)  since base match 0.0m  match 0.70 (avg 0.70)
```

让 agent 去冰箱拿一瓶可乐，它听得懂要做什么，却不了解这个家：冰箱在哪、厨房怎么走、可乐上次放在哪。无论用哪个模型，agent 都需要空间记忆：能根据一句命令、一段文字或一张照片记住一个地点，之后让机器人回到那里，并且这份记忆在上下文压缩、地图重新优化和进程重启之后依然有效。EvergreenSLAM 是围绕这个需求设计的 2D 激光 SLAM。

- **单一模式，无需人工操作。** 系统作为后台进程常驻，不区分建图模式和定位模式：始终在当前 *session* 中建图，同时在已*冻结*的 session 上定位；一个 session 新增的区域足够多、位姿约束足够好之后，会自动冻结。
- **资源随面积增长，不随运行时长增长。** 删减时对被删部分做边缘化，再将结果稀疏化为 Chow-Liu 树；冻结位姿在优化中保持为常量。因此内存和 CPU 的上界由建图面积决定，与运行时长无关。
- **地点长期有效。** 地点是*锚点*，绑定在位姿图的子图上，而不是坐标上；agent 通过一棵目录树和一个小 CLI（`egs`，也提供 MCP 工具）来使用。
- **只需一颗廉价雷达。** 一颗 2D 雷达加 CPU 即可，不需要 IMU、轮式里程计、相机或 GPU。已在 215 元的雷达和 RK3588 板子上做过实机测试，也在公开 2D 数据集上测试过。

扫描匹配和子图沿用 Cartographer 的设计，其上的 lifelong 层由本项目实现。与 slam_toolbox 的对比见「相关工作与致谢」。

<p align="center">
  <img src="images/car.jpg" width="640" alt="测试小车：一颗 2D 雷达装在 Orange Pi 上，麦轮，两个充电宝">
  <br>
  <sub>测试平台：一颗 2D 雷达（215 元，约 30 美元），一块 RK3588 的 Orange Pi，麦轮，两个充电宝。</sub>
</p>

## 动机

**agent 不了解这个家。** 语言理解已经不是瓶颈：让 agent 去冰箱拿可乐，它明白要做什么，缺的是这个家的布局。agent 的记忆是文本，把布局描述到足以据此行动的程度，要占用大量上下文，而且只在上下文存续期间有效，也经不住上下文压缩。

**坐标很脆弱。** 用地图坐标记录地点维持不了多久：地图重建或重新优化后坐标就会失效，agent 往往察觉不到；换一张地图，坐标就毫无意义。基于常规 SLAM 的 agent 都得自己维护一套「东西在哪」的记录，还要让它和地图保持一致。agent 需要的是一个始终指向同一物理位置的*名字*，以及一个记录在那里所见内容的地方。

**无人值守的 SLAM。** 常见的 SLAM 流程是：调里程计、标定外参、同步时间戳，开着车建图，存图，切到定位模式，环境变了调参，变多了重建。厂家通常会把这套流程封装好，或者培训客户；新手一旦哪一步出错，地图就会出现重影或漂移，有时到后面才发现，而重建意味着所有站点都要重新配置。每一步都需要人来操作，而 agent 不应该被迫充当这个操作员。

**设想的使用方式。** 例如：带着机器人在家里走一圈，到厨房时说「这里是厨房」，到书桌时说「这是书桌」，此外不再处理任何建图或定位的事。第二天说「去厨房看看」，它就知道怎么去。agent 记下剪刀在书桌左边的抽屉里、苹果在餐桌上，几天后问「剪刀在哪」，它能找到对应的地点。家具挪了、地图重新优化了、机器人重启了，这些地点依然有效。拿可乐这件事，也就从「知道冰箱在厨房、知道厨房怎么走」开始，到了再交给机械臂。后面这些步骤还需要机械臂和视觉配合；最底下这一层，即无需人维护地图、始终有效的地点，本项目认为应当由 SLAM 直接提供。下文的演示在实机上、从空地图开始验证了其中与记忆有关的一半：探索中记住地点，为每个地点写笔记，再凭这份记忆重新找到一个地点和一件东西。用机械臂取物不在演示范围内。

**lifelong SLAM 在哪。** 这些需求的底层就是 lifelong SLAM：一直开着，边用边建，跟随环境变化，资源不随运行时长增长。这个方向的研究文献很多，但据作者所知，能直接拿来使用的开源 2D 实现很少；slam_toolbox 的 lifelong 节点在其 README 中标注为 highly experimental，删除节点时也不做边缘化（见「相关工作与致谢」）。

**成本。** 这份记忆得便宜到可以在小车上一直开着。3D 激光几百美元起步，还得配一块跟得上它数据量的板子。摄像头便宜，但稠密的或基于学习的视觉 SLAM 通常要的算力比一块小板子多，而且摄像头在黑暗、无纹理的墙面和光照变化下会退化。2D 激光两百块钱左右，不需要 GPU，不受光照影响；它自身的失效场景（玻璃、镜子、强光直射）本项目不处理。出于同样的理由，管线不融合 IMU、轮式里程计和摄像头：每多一种传感器就多一套标定、时间同步、驱动和按车调参。代价是运动预测只能依靠激光：一个采用 Singer 运动模型、只以扫描匹配结果为输入的卡尔曼滤波，在快速转弯和缺少特征的长路段上预计表现不佳。本项目的取舍是：廉价的几何信息，加上挂在命名地点上的稀疏语义（比如让视觉语言模型偶尔记一句"垃圾桶在这"），在这个场景下比稠密视觉 SLAM 更划算。

## 工作原理

### session 与冻结

<p align="center">
  <img src="images/demo.gif" width="960" alt="revo_lds 数据包在浏览器调试视图里的回放：地图生长，session 0 和 1 先后冻结，session 2 开始建图">
  <br>
  <sub><code>pixi run -e ros demo</code>：Cartographer 公开的 revo_lds 数据包（Neato 扫地机上的 Revo LDS 雷达，约 16 分钟）在浏览器调试视图中 8 倍速回放。session #0 和 #1 先后冻结，随后 #2 开始建图。</sub>
</p>

建图以 session 为单位组织。前端始终在当前 session 中建图；后端对可达范围内所有已完成的子图做回环，无论是否冻结，也包括当前 session 自己的子图，而下文的滚动窗口保证这些子图都是最近建的。环境逐渐变化时，机器人匹配的是当前的环境，而这些最近的子图又被约束到冻结的底图上。

第一个 session 有了已完成的子图后即成为冻结的底图。之后的 session 要先在冻结区域之外新建足够多的区域、并且停止增长，才会进入冻结判断；当它与某个已冻结 session 建立了约束，且每张已完成子图位姿的边缘标准差在各轴上都低于阈值时，就会冻结。冻结后，该 session 的文件变为只读，位姿在之后的所有优化中保持为常量。只在已知区域内活动的 session 不会自动冻结（可用 `egs session freeze --yes` 手动冻结）；未冻结的 session 若超过五次启动都没有新数据输入，就会被丢弃（可配置）。session 之间重叠的区域，合成地图取置信度更高的格子。因此，已建区域内的变化在当前 session 存续期间可以看到，但不会被冻结进底图；对变化后的房间重新冻结已在计划中，尚未实现。

### 重定位

`egs init-pose`（给位姿或已存的地点）和 `egs relocalize` 拿当前 session 最新的一帧扫描，去匹配其他 session 的已完成子图：给了位姿时，取覆盖到它 2 m 以内的子图，角度范围 ±0.5 rad；没给时，取全部子图、全部朝向，逐张做分支定界。匹配得分超过常规阈值才成为回环约束。还没连上冻结 session 的 session，本身的位置没有保留价值，所以没加上约束时，它会被整体移到得分不低于 0.2 的最佳匹配上，没有这样的匹配就移到给定的位姿。之后它仍处于未连接状态，直到有回环通过常规阈值；`egs` 把这种结果报告为 `reseeded`。已连上底图的 session 不会被这样移动。

### 有界增长

两个机制让图不随运行时长增长。

*带稀疏化的删减。* 活动 session 在重复经过的区域只保留自己子图的一个滚动窗口：旧子图的区域已被更新的子图覆盖，或其轨迹已被之后的几趟覆盖时，就会被删除。删除通过边缘化完成，而不是直接丢弃：取被删节点的 Markov 毯，计算保留节点的联合协方差，再将得到的稠密信息稀疏化为一棵 Chow-Liu 树（基于两两互信息的最大生成树），用树的边替换被删除的约束；跨度过大的边转为按边缘协方差加权的先验。冻结时，大部分区域已落在冻结地图内的已完成子图也按同样方式删除；若全部满足条件，则保留一张。

*冻结的 session 是常量。* 冻结位姿在 Ceres 问题中是设为常量的参数块，两张冻结子图之间不建约束，冻结的子图也从不修改。因此优化规模只取决于活动 session，与整张地图无关；冻结部分随建图面积线性增长。仿真中 30 圈的运行保持有界。实机上已在一套住宅里运行过数天，但当时没有记录内存和位姿图规模，数据之后补充。

<p align="center">
  <img src="images/webui.jpg" width="900" alt="浏览器调试视图：子图、轨迹和四个 session，其中三个已冻结">
  <br>
  <sub>长时间运行时的浏览器调试视图：四个 session，其中三个已冻结。</sub>
</p>

### 地点即锚点

保存的地点是一个锚点：位姿图中的一张子图加上一个相对变换。图重新优化时，地点随子图一起移动；重启后，名字依然有效。锚点在回环、删减、冻结、重启以及写入中断之后都能保持，`egs where` 查询的就是它。

### 记忆即目录

`memory/places/kitchen/table/apple` 表示某个地点中的一个物体。agent 用 `ls`、`cat`、`mv`、`grep` 操作这棵树。只有运行中的进程才掌握的信息，比如位姿、保存地点、地图和 session 管理，才通过 `egs`。禁区、笔记和各地点的 skill 都在同一棵树里。`egs view here` 把当前扫描叠加在机器人周围的地图上渲染出来，供 agent 自行判断定位是否漂移。

### 实现

纯 2D，全部 `double`，位姿用 `Eigen::Affine2d`。激光里程计用相关性匹配加 Ceres 精修，子图是概率栅格，位姿图用分支定界回环加 Ceres 优化，在此之上是按 session 进行的删减、冻结，以及原子提交的持久化。`core/` 不依赖 ROS，ROS 2 只是其中一个适配器。

## 实验

<p align="center">
  <img src="images/robot.jpg" width="320" alt="演示中的机器人：装在三层推车上的 XLeRobot，全向轮，头部相机装在颈部，两条机械臂">
  <br>
  <sub>演示所用的平台：一台 XLeRobot（三个全向轮、装在云台上的头部相机、两条 SO-101 机械臂，未使用），雷达与上面的小车相同，主控是一块 RK3588 板子。EvergreenSLAM 只用到其中的雷达和板子。</sub>
</p>

### agent 探索、记录、回访

<p align="center">
  <a href="https://youtu.be/FXoj2yCrbuo"><img src="https://img.youtube.com/vi/FXoj2yCrbuo/maxresdefault.jpg" width="640" alt="YouTube 上的完整演示视频（3 分 18 秒）"></a>
  <br>
  <sub>完整视频（3 分 18 秒，英文字幕），点击在 YouTube 观看。</sub>
</p>

**演示任务**：

演示任务使用 Claude Code（Opus 5.5 模型）作为上图机器人的「大脑」先后完成两次会话：当天 24 分钟，第二天早上 4 分钟。机器人的身体由一套不在本仓库中的机器人 harness（暂未公开）经 MCP 提供以下功能：语音识别、跟随人、用 Nav2 导航到某个位姿或已存的地点、转头、相机拍照、雷达扫描，以及用 Telegram 回消息和发照片。本仓库的 `egs` 提供它唯一的空间记忆。

第一天有意从一张新地图和空的 `memory/` 开始，展示的是 agent 怎样建立和使用记忆。第二天早上，重新启动机器人（无需人工关心和操作定位）和一个新的 agent 会话，展示的是跨重启、跨会话的空间记忆；更长期的 SLAM 运行实验见「session 与冻结」和「有界增长」。使用者全程使用中文自然语言，agent 用英文思考和写笔记，地点的中文名写在别名里。

> 实际上这次录制并不顺利：视觉服务中途掉线，探索时漏掉了放剪刀的那张桌子，agent 前后问了我四次，在我提示「剪刀一般放在桌子上」之后，它去了地图上还没探索过的区域才找到。之前两次试验里，探索都经过了那张桌子，一句「帮我找一下剪刀」两分钟左右就找到了。目前的效果还很依赖模型能力和 harness 的设计。

**过程记录**（时间为视频中左上角显示时间）：

1. **开始（17:03）。** `egs root`、`egs fs cat README.md`、`egs here`、`egs fs tree places`：新地图，没有任何地点。
2. **人指定的唯一地点（17:03–17:06）。** 「跟着我」，然后「你的休息点以后就在这个冰箱的侧边」；经过几次语音微调后执行 `egs place save places/rest-spot`，再写 `node.yaml`（别名）和 `notes.md`，记明这是人告诉它的。整个会话中人只指定了这一个地点。
3. **独自探索（17:06–17:12）。** 收到「你先在家里面随便探索一下」后，agent 从 `egs view map` 里挑没去过的区域，让 Nav2 开过去，用头部相机观察，把发现的东西存成地点：手办架、客厅和沙发、洗衣机、电脑桌、卧室（存在门口）。每个 `notes.md` 都写明那里有什么、信息从哪来：亲眼看到、人告诉的，还是推断的。17:08 根据一张墙角的照片，它在客厅笔记里写了一行：「a red striped curtain and, past it, a glass door to what looks like a balcony (阳台, inferred)」（红色条纹窗帘后面有一扇玻璃门，看起来通往阳台，推断）。阳台既没去过，也没有保存。
4. **为地点写下的规则（17:13）。** 回休息点停靠时，Nav2 在最后几厘米来回蠕动；agent 让它停下，自己用几次短距离移动收尾，然后写了 `places/rest-spot/SKILL.md`：怎样收尾（冰箱侧板正对身后，雷达后方扇区读数 0.43 到 0.45 m）。之后 `egs where places/rest-spot` 会把这个文件列在 `read first:` 下。
5. **「你知道剪刀在哪吗」（17:12–17:23）。** `egs find 剪刀` 和 `egs fs grep scissors` 都没有结果。harness 的物体检测服务从 17:08 起就不再响应；agent 在第一次超时后就改为自己看全分辨率快照，并在开始找剪刀时说明了这一点。它依次排查了已知的候选：电脑桌、客厅里放胶带和小零件的架子（存为 `tool-shelf`）、玄关和卧室的小桌子，一路保存新地点，并把每处「没找到」追加到该地点的 `notes.md`。期间人说了四次话：「你去找找吧」（回答它的提问）、「再去其他地方找一下呢」、一次提示「剪刀一般放在桌子上」（它把这句作为人告知的习惯写进 `places/notes.md`），以及「你自己去找」。之后它去了地图上显示有障碍物、但还没去过的区域，即客厅和玄关之间，在一张小边桌上找到了剪刀：`egs place save places/side-table`、`egs observe places/side-table/scissors --offset …`，17:23 发出照片，距提问约十分钟。探索阶段没有走到那个角落。这是当天三次找剪刀里最慢的一次；另外两次探索时已经经过那张桌子，没有提示，约两分钟找到。这次的时间主要花在先排查的四个地方和行驶上，视觉服务失败的调用每次约一秒就返回，影响很小。
6. **「去阳台给我拍个照，然后就回休息点吧」（17:23–17:25）。** `egs find 阳台` 没有匹配：没有任何东西叫阳台。`egs fs grep balcony` 返回第 3 步那行推断。agent 开到 `places/living-room/tool-shelf`（剪刀搜索时存的），把头转向窗帘，移动到入口处，而不是从沙发旁硬挤进去，保存 `places/balcony`（别名 阳台、封闭阳台、balcony；笔记写明是人起的名字），17:25 发出照片。从听到请求到发出照片 102 秒，期间人没有再说话。
7. **回到休息点（17:25–17:26）**，按自己写的规则收尾：到达时偏 3 cm，用雷达检查，再后退 5 cm。
8. **第二天早上（2026-11-01，09:39–09:43）。** 机器人在停下的位置断电过夜。开机后没有执行 `egs init-pose` 或 `egs relocalize`，`egs here` 即报告它在 `places/rest-spot`（0.0 m），与底图的一致度 0.73、覆盖 90%。一个没有对话历史的新 Claude Code 会话收到四行提示词：读同样的三个 skill，只用 `egs` 里的记忆。（Claude Code 同时自动加载了 harness 的项目说明，其中把「找剪刀」列为目标任务，但没有任何物体的位置。）人在终端里输入「你还记得剪刀在哪吗，去拍张照给我」。`egs find 剪刀` 返回 `places/side-table/scissors`，最后一次看到是 16 小时前；agent 说要去核实，而不是直接相信旧的观察记录，用 `egs where` 给出的接近位姿开过去（12 秒）。物体检测服务没有运行，它按记忆里的偏移转头，自己判断快照，在读到问题 44 秒后发出照片（这条消息在 Claude Code 的输入队列里等了 46 秒，因为当时正在执行一个 `listen` 调用），并用 `egs observe` 追加了一条新的 `seen` 观察。随后让它回休息点，它按前一天自己写的 `places/rest-spot/SKILL.md` 收尾：离冰箱 0.49 m，规则是 0.43–0.45 m，于是短距离后退。这只是一次重启，机器人在断电期间没有被移动；它不能说明机器人被搬到别处后的重定位能力。

第一天结束时，`memory/` 里有 12 个地点、一个物体（剪刀，带观察记录）、一条地点规则和一份全屋笔记，全部是普通文件。第二天 agent 只追加了一条观察记录。

`egs` 负责回答地点在哪（`where` 给出位姿和接近位姿，由 harness 交给 Nav2），把保存的地点绑定到位姿图上，并存放这些文件（`fs`、`find`、`observe`、`view`）。行驶、避障和停靠由 Nav2 和 harness 完成；听、看、识别照片里的东西以及所有决定，由 agent 借助 harness 的工具完成。第一天的提示词要求 agent 读三个 skill（本仓库的 `evergreenslam`，以及 harness 的两个 skill：一个讲怎样移动机器人，一个讲该记什么，包括来源标注、把地点规则写进 `SKILL.md`、记下没找到的地方），每次做决定前说一句理由，窄处不要硬挤，人要的照片同时存进记忆。手柄一直在人手里，只作急停；机器人底层守护进程的日志显示两次会话中都没有遥控租约。

### 更早的实机测试

2026-09-29，在更早的一台测试小车上（麦轮、2D 雷达、RK3588 板子），Claude Code 收到「去厨房看看有没有变化」。它读了记忆目录里厨房的 `SKILL.md`（停在坡上面的门口），把 `egs where places/kitchen/door` 给出的接近位姿交给 Nav2，途中被空气净化器卡住一次后退脱困，最后用 `egs observe` 记下垃圾桶旁多出的一个东西。这台小车是第一个公开版本（2026-09-30）的测试平台；2026-10-07 起由上图的机器人接替，此后该机器人一直运行 EvergreenSLAM 作为空间记忆。

## 验证与局限

本项目目前是预览版，不是成熟产品。范围限定在核心功能，已知缺项如下。

在 macOS 和 Linux 上测过：单测；前端、扫描匹配器和后端每一级（回环、删减、冻结、持久化、重定位、锚点）的模拟激光端到端测试。每次 push 由 CI 在 Linux（gcc）和 macOS 上跑 core 与 agent 的测试；后端测试的 ThreadSanitizer 和 ROS 2 适配器的构建仍是手动。在上述两台机器人和公开数据包上验证过：激光里程计、回环、多 session 建图与自动冻结和删减、原子提交的持久化和命名地图、agent 服务和 `egs`、浏览器调试视图、rviz2 插件。数据集上回放工具会报告相对参考轨迹的误差（见 ROS README）；这里还没有整理成结果表。

- 不融合 IMU、轮式里程计和摄像头，没有 3D（见「动机」）。
- 冻结的 session 不能解冻。已建图区域内的变化会由当前 session 建图和匹配，它通过与冻结底图之间的回环保持对齐，因此定位不受影响；但这些变化不会写进冻结地图（见「session 与冻结」）。如果地图发散后又被冻结，只能换一个地图目录。
- 没有动态障碍物滤除。里程计无法处理几何退化场景（长走廊、空旷大厅）；回环会拒绝走廊中有歧义的匹配。回环不检查扫描是否与地图中的空闲区域一致，因为在旧地图上，挪动的家具和开关的门会让正确的匹配也落进空闲区域；因此得分高的墙角或平行墙混叠会被接受，只靠鲁棒核函数限制它的影响。
- 一台机器人一张图。不支持多机。
- SLAM 本身不检测机器人被搬走；agent 从 `egs here` 的 `base` 分数看出来，再用 `egs init-pose` 或 `egs relocalize` 处理。这是有意交给 agent 的：两间相似的房间里，agent 看门牌、报出地点，比盲目的全局搜索更可靠，后者可能锁到错误的房间。
- `egs relocalize` 对每张子图做完整搜索，开销随地图增长：46 张子图在 Apple M4 的四个线程上需要 13 到 16 s，期间回环要排在它后面。更快的全局搜索已在计划中。
- 一个已知的里程计缺陷：模拟房间里从静止沿着与墙平行的轴直线跑，曾观察到位姿一直停在原点。对应的回归测试暂时禁用，待查明原因。

只在一套住宅和公开数据集上评估过。欢迎开 issue。

## 编译与运行

只编 core，不需要 ROS：

```bash
cmake -S core -B build -DEVERGREENSLAM_BUILD_TESTS=ON && cmake --build build -j8 && ctest --test-dir build
```

依赖：Eigen、Ceres、glog、yaml-cpp、protobuf（库和 `protoc`）、OpenMP、gtest。

```bash
# macOS
brew install eigen ceres-solver glog yaml-cpp protobuf libomp googletest
# Ubuntu 24.04（core 链接 yaml-cpp 0.8，22.04 的包太旧）
sudo apt install libeigen3-dev libceres-dev libgoogle-glog-dev libyaml-cpp-dev \
                 libprotobuf-dev protobuf-compiler libgtest-dev
```

或者让 [pixi](https://pixi.sh) 把一切装好，包括 ROS 2 Jazzy 和 rviz2（macOS Apple Silicon 和 Linux；`pixi.toml` 头部列出了全部任务）：

```bash
pixi run core-test                      # 只有 core
pixi run agent-test                     # agent 服务和 egs
pixi run -e ros ros-test                # ROS 2 适配器
SCAN_TOPIC=/base_scan BASE_FRAME=base_footprint pixi run -e ros bag bags/wg_cafe   # 回放，浏览器视图在 http://localhost:8642
pixi run -e ros live                    # 实时节点：/scan 进，地图、tf 和 agent 服务出
pixi run -e ros demo                    # 下载 3 MB 的公开数据包，带浏览器视图和 egs 回放
```

`live` 需要一个雷达驱动往 `/scan` 发 `sensor_msgs/LaserScan`（`SCAN_TOPIC` 可改），以及 `base_link` 到雷达坐标系的 TF（两者不同时）。它总会用激光里程计发 `/odom`（不带协方差），开着 `publish_tf` 时还发 odom 到 base_link 和 map 到 odom 两个 TF；底盘驱动如果也发 odom 到 base_link，应关闭底盘那边的，因为 `publish_tf=false` 会把 map 到 odom 一起关掉；两个 `/odom` 话题也要 remap 掉一个。配置从 `configs/evergreenslam.yaml` 起步，museum 和 carmen 是给 museum 和 CARMEN 数据集用的。pixi 下地图存在 `runs/maps`（`MAP_ROOT` 可改；`bag` 只在设了 `MAP_ROOT` 时才落盘）。pixi 只配了 osx-arm64 和 linux-64；Linux ARM 板子上用系统包或 colcon。在 ROS 2 Jazzy 上测过。

仓库本身就是一个 colcon 工作空间源：放进任意 `ws/src`，然后 `rosdep install --from-paths src --ignore-src -y && colcon build`。Docker Compose 在 Linux 上封装了同样的目标（`docker compose run --rm core-test`）。

没有实时时钟（RTC）的板子上，节点应在时间同步之后启动：systemd unit 里写 `After=time-sync.target` 和 `Wants=time-sync.target`，并启用时间服务的等待单元（`chrony-wait.service` 或 `systemd-time-wait-sync.service`）。扫描时间戳倒退，或向前跳过 `max_scan_gap_seconds`（默认 60 s），都按时钟跳变处理：节点保存地图后以非零状态退出，重启时从上次保存的位姿重新对齐。因此需要配一个守护进程：systemd 的 `Restart=on-failure`，或 ROS 2 launch 的 `respawn`（`evergreenslam.launch.py` 已设置）。回放工具遇到同样情况会报错停止。

### 数据

仓库不带数据。公开的 2D 激光包大多是 ROS 1 的，两个脚本不装 ROS 也能转：

```bash
pip install rosbags
curl -O http://download.ros.org/data/graph_slam/wg-cafe.bag
python3 tools/convert_ros1_bag.py wg-cafe.bag bags/wg_cafe
SCAN_TOPIC=/base_scan BASE_FRAME=base_footprint EXTRA="--speed 4" pixi run -e ros bag bags/wg_cafe
```

开着浏览器视图时回放按真实时间走，`EXTRA="--speed 4"` 加速。回放时 agent 服务只在设了 `MAP_ROOT` 且加 `EXTRA="--agent_port 8643"` 时才开。

经典数据集（Intel Research Lab、ACES、MIT Killian Court）是 CARMEN 日志，在 [Freiburg SLAM evaluation 页面](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php)；`tools/convert_carmen_log.py` 把 `.clf` 转成包。回放工具怎样对照参考轨迹打分，见 [`adapters/ros2/evergreenslam_ros/README.md`](../adapters/ros2/evergreenslam_ros/README.md)。

### agent 接口

实时节点跑起来后，agent 服务监听 `127.0.0.1:8643`，`egs` 是客户端：

```bash
export PATH="$PWD/adapters/agent/egs/bin:$PATH"
egs help agent                     # 规则和命令表，不到 30 行
egs here                           # 机器人在哪
egs status                         # 地图健康：冻结底图、对齐、回环
egs place save places/kitchen/door # 记住脚下这个位置
egs where places/kitchen/door      # 位姿、精度，以及接近它的位姿
egs view here                      # 扫描叠在地图上，机器人周围
```

`egs` 是 Python 3.9+，不依赖任何第三方库；`pipx install ./adapters/agent/egs` 可装在没有 ROS 的机器上，`egs skill install <skills 目录>` 给 agent 装 skill。skill 的第一节就是上手步骤（`egs root`，用 `egs fs cat` 读 `memory/README.md`，读到该地点为止的 `SKILL.md` 链）。只接工具、不接 shell 的 agent 可用 `egs mcp`：它以 stdio MCP 服务提供同一组命令，工具由 CLI 定义自动生成，与命令一一对应。服务只绑 `127.0.0.1`；agent 在另一台机器上时经 SSH 隧道访问（`ssh -NT -L 127.0.0.1:8643:127.0.0.1:8643 user@robot`），连接设置写在 `~/.config/egs/config`（见 [`adapters/agent/egs/README.md`](../adapters/agent/egs/README.md)）。HTTP 契约见 [`adapters/agent/API.md`](../adapters/agent/API.md)，skill 是 [`adapters/agent/skills/evergreenslam/SKILL.md`](../adapters/agent/skills/evergreenslam/SKILL.md)。

### 目录

```
core/            SLAM 本体，无 ROS：sensor / mapping / lifelong（位姿图、session、持久化）/ utils
adapters/ros2/   三个 ament 包：msgs、节点与回放工具、rviz2 插件
adapters/agent/  agent 服务（HTTP）、egs CLI 和 agent skill
apps/webui/      浏览器调试视图
configs/         yaml 配置；configs/reference.yaml 列出全部键
tools/           包转换脚本、TSan 门禁和 demo
```

## 相关工作与致谢

- [Cartographer](https://github.com/cartographer-project/cartographer)（Hess 等，2016；Apache 2.0）。相关性扫描匹配器、预计算栅格、概率栅格子图、体素和运动滤波、约束构建器沿用它的设计，rviz2 的子图显示移植自 `cartographer_rviz`。见 `NOTICE`。`core/src/utils/scan_matching` 离原版最近。Cartographer 没有 session、冻结和带边缘化的删减，这些由本项目实现。
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)（Macenski 与 Jambrecic，2021），Nav2 支持的 2D 激光 SLAM。它的模式由用户通过节点和配置选择：建图（同步或异步）、在序列化的位姿图上继续建图，以及用最近扫描的滚动窗口在序列化的位姿图上定位；另有一个实验性节点，可通过服务调用在建图和定位之间切换。删除节点的地方有两处，都经过 `Mapper::RemoveNodeFromGraph`，它把节点连同它的边一起删掉，不做边缘化：定位模式删除自己过期的扫描；`src/experimental/` 下的 lifelong 节点（其 README 称之为高度实验性）按每帧新扫描给附近的地图节点打分，删掉低分节点。它要求 TF 中有 odom 到 base 的变换：没有这个变换的扫描会被丢弃，最小移动距离的判断也在这个坐标系里做。EvergreenSLAM 不区分建图与定位，自行决定 session 何时冻结，删减时做边缘化，也不需要里程计输入。slam_toolbox 的扫描匹配器来自 Karto，其 README 称 Karto 的改进版本在世界各地的公司中使用；此外它还有 Nav2 集成、多机建图、地图合并和 rviz 中的交互式图编辑。以上对照的是其 `ros2` 分支的 6efdc33 提交（2026-10-15）。
- 删减中的边缘化和 Chow-Liu 稀疏化沿用位姿图压缩方向的文献，思路与 Kretzschmar 和 Stachniss（2012）相同。
- [Claude Code](https://claude.com/claude-code)。大部分代码是用 Claude 写的，设计、审查和测试由作者完成。
- [cpp-httplib](https://github.com/yhirose/cpp-httplib)（MIT）和 [stb_image_write](https://github.com/nothings/stb)（公有领域），放在 `third_party/`。
- [RoboStack](https://robostack.github.io/)，它把 ROS 2 装进了 pixi，macOS 上的开发流程依赖它。
- Freiburg 和 Willow Garage 的数据集，回放和打分用的是它们；Cartographer 的 revo_lds 数据包，demo 用的是它。

## 许可

Apache License 2.0，见 [`LICENSE`](../LICENSE) 和 [`NOTICE`](../NOTICE)。
