# Accel-Sim（GPGPU-Sim）Shared Memory → Ramulator2（Mono3D/BankParallel）建模说明

> 目的：本文档描述**目前已经实现**的“3D 堆叠 DRAM shared memory（v0）”在 Ramulator2 中的表达方式、上层 Accel-Sim 的调用方式、关键参数与统计口径，方便主人在上层做配置/对接与结果解读喵～ (..•˘_˘•..)

> 一句话总结：当前已提供 `DRAM.impl: Mono3D`（v1）设备模型，并与 **`GenericDRAMSystem + BankParallel 控制器`** 组合用于 Accel-Sim shared-memory 近存建模；`layer -> channel`、`bank_parallel_ports_per_layer`、row buffer 统计口径与 v0 保持一致，区别在于 DRAM 设备/timing 不再使用 DDR4 占位。

## 1. 总体抽象与建模边界

### 1.1 目标关注点

当前模型聚焦两类冲突/并行度约束：

- **Row conflict / row buffer 行冲突**：同一 bank 的 row buffer 命中/失配/冲突（Ramulator2 内建统计）。
- **Bank conflict / bank 端口冲突**：同一 layer 内 bank 级并行访问的“端口数/issue 宽度”限制（本项目新增 `BankParallel` 控制器）。

### 1.2 Mono3D 的简化形态（当前版本）

我们把“3D 堆叠 DRAM 贴近 SM，作为 shared memory 后端”的形态，先抽象为：

- 多个 **layer**（不考虑 z 向非均匀延迟：各 layer latency 相同）。
- **layer 间可并行**。
- 每个 layer 内有多个 **bank**，bank 间可并行。
- bank 内采用传统 DRAM 简化：包含 **row buffer**，受 ACT/RD/WR/PRE 等时序约束（具体 timing 由 DRAM 设备模型与 timing preset 决定，后续可替换为 Mono3D 参数）。

### 1.3 “方案一”：layer → channel

为便于在 Ramulator2 里落地，采用映射：

- **一个 layer 对应 Ramulator2 的一个 channel**；
- 因此 `DRAM.org.channel = layer_count`；
- `BankParallel` 的 `bank_parallel_ports_per_layer = P` 代表**每个 layer（每个 channel）每周期最多并行推进 P 个 bank 的访问命令**（详见 3.2）。

> 重要：当前在 Accel-Sim shared-memory backend 的用法里通常是“每个 SM 一个 Ramulator2 memory system 实例”，因此不同 SM 间不会互相争用该 Mono3D shared memory（这符合“贴近 SM”的近存假设）。

## 2. Accel-Sim 调用路径与数据流

### 2.1 组件关系

Accel-Sim（shared memory 请求）到 Ramulator2 的数据流如下：

```
GPGPU-Sim shared-memory 访问
  -> ShmemBackendRamulator2 (每个 SM 一个实例)
      -> Ramulator2 Frontend: GEM5 (external requests)
          -> MemorySystem: GenericDRAMSystem
              -> AddrMapper: 把物理地址映射为 addr_vec(levels)
              -> Controller[channel]: BankParallel (每个 channel/layer 一个)
                  -> DRAM device model: DDR4 / (未来 Mono3D)
```

关键实现文件：

- Accel-Sim 侧适配器：`gpu-simulator/gpgpu-sim/src/gpgpu-sim/shmem_backend_ramulator2.cc`
- Ramulator2 memory system：`third_party/ramulator2/src/memory_system/impl/generic_DRAM_system.cpp`
- Ramulator2 addr mapper：`third_party/ramulator2/src/addr_mapper/impl/linear_mappers.cpp`
- 新增控制器：`third_party/ramulator2/src/dram_controller/impl/bank_parallel_dram_controller.cpp`
- Mono3D v1 设备模型：`third_party/ramulator2/src/dram/impl/Mono3D.cpp`

### 2.2 Accel-Sim 的配置入口（上层调用）

在 `gpgpusim.config`（或你使用的配置文件）中启用：

- `-gpgpu_shmem_backend ramulator2`
- `-gpgpu_shmem_backend_config <yaml路径>`

示例 YAML（用于 shared-memory backend）在：

- `gpu-simulator/gpgpu-sim/configs/shmem_ramulator2_bank_parallel_example.yaml`（DDR4 占位 + BankParallel）
- `gpu-simulator/gpgpu-sim/configs/shmem_ramulator2_mono3d_bank_parallel_example.yaml`（Mono3D v1 + BankParallel）

## 3. Ramulator2 侧“当前 Mono3D 简化模型”如何表达

### 3.0 当前已实现的模型（v1）：3D-SHMEM-Mono3D（精确到代码行为）

为了避免“名不副实”，这里把**当前真实实现**按组件拆开说明（主人上层对接时就按这份行为假设来喵）：

#### 3.0.1 组件与实现点（v1）

- MemorySystem：`MemorySystem.impl: GenericDRAM`（`third_party/ramulator2/src/memory_system/impl/generic_DRAM_system.cpp`）
  - 每个 `channel` 创建一个 controller 实例（controller 的 `id` 会标记为 `Channel i`）。
- Controller：`Controller.impl: BankParallel`（新增，`third_party/ramulator2/src/dram_controller/impl/bank_parallel_dram_controller.cpp`）
  - **每个 tick 最多 issue `P` 条命令**（`P = bank_parallel_ports_per_layer`）。
  - 在同一 tick 内，对 “访问类命令”（`is_accessing`，一般对应 RD/WR）施加 “**同一 bank 只能被访问一次**” 的并行约束。
  - 其它行为（read/write buffer、write-mode 切换、水位线、scheduler/rowpolicy/refresh/plugin 调用顺序、row hit/conflict 统计口径）基本沿用 Generic controller 的结构。
- DRAM 设备模型：`DRAM.impl: Mono3D`（新增，`third_party/ramulator2/src/dram/impl/Mono3D.cpp`）
  - 含义：row buffer 行状态、ACT/RD/WR/PRE 的时序与约束来自 Mono3D；可通过 YAML 的 `org`/`timing` 覆盖参数。
- Frontend（Accel-Sim 场景）：强制使用 `Frontend.impl: GEM5`（external request wrapper）
  - 这是 `gpu-simulator/gpgpu-sim/src/gpgpu-sim/shmem_backend_ramulator2.cc` 在 real-mode 下固定创建的 frontend，用于接收上层外部请求。

#### 3.0.2 v1 版本到底“建模了什么” vs “没建模什么”

已建模（对主人上层最重要的部分）：

- Layer 并行：通过 `channel = layer` 映射实现（只要地址映射能把请求分散到不同 channel，就能并行推进多个 layer）。
- Bank 级并行上限：用 `bank_parallel_ports_per_layer = P` 表达（每 layer 每周期最多 issue P 个命令，且访问类命令必须落在不同 bank）。
- Row buffer 行命中/冲突：来自 DRAM 设备模型（row open/row hit/row conflict 的判定函数是 `IDRAM::check_rowbuffer_hit` / `check_node_open`）。

暂未建模（v1 仍刻意不做，以免过度设计/YAGNI）：

- Z 向 tier/layer 非均匀延迟（我们假设各 layer latency 相同）。
- TSV/stack 内互连细节、层间带宽瓶颈、跨 layer 资源争用。
- 近存计算中的“在 DRAM 内部算”指令/数据通路（目前只模拟 memory access）。
- Shared memory 传统“bank conflict 解析式时序”与 Ramulator2 的联合（Accel-Sim 这边已避免双计数，原则是：启用 backend 时由 backend 给出时延）。

#### 3.0.3 BankParallel 的 issue/冲突判定细节（建议主人重点阅读）

BankParallel 在每个 controller tick 内做：

1. 维护一个本周期已使用的 `used_access_banks` 集合（仅记录访问类命令占用的 bank）。
2. 重复最多 `P` 次：
   - 从 active/priority/read/write buffer 中“挑一个最优请求”（按 scheduler 比较），但会跳过那些会违反 bank 并行约束的候选；
   - 若该请求的当前 `preq_command` 就绪，则 issue；否则继续寻找/或停止。
3. 若 issue 的命令是访问类（`is_accessing`），将该请求的 bank key 记录进 `used_access_banks`。
4. 若该命令是请求的 `final_command`：
   - Read：在 `m_read_latency` 后回调完成；
   - Write：1 cycle 后回调完成。

bank key 的定义（用于“同一 bank”判定）：

- 取 `addr_vec` 从 level 0（channel）到 bank level（包含 bank）的前缀拼成键。
- 因此两个请求只要属于同一 channel 且同一 bank，就会被视为冲突（同周期只能并行一个访问类命令）。

> 重要：当前 v0 的 bank 并行约束只对 `is_accessing` 生效；ACT/PRE 等命令**不会占用 “本周期已用 bank” 的冲突集合**，这会在某些激进调度下略偏乐观。

补充两个容易混淆的点（以代码为准）：

- **issue 预算 `P` 是“每周期最多 issue 的命令条数”**：无论是 ACT/PRE 还是 RD/WR，只要被 issue，就会消耗一次 `P` 的预算。
- **bank 冲突约束只对访问类命令生效**：只有 `is_accessing==true` 的命令才会占用 “同一 bank 本周期已用” 的集合。

#### 3.0.4 请求队列、背压与 read/write 模式（v1 默认值）

BankParallel 控制器内部有 4 类 request buffer：

- `m_active_buffer`：正在服务/已打开行的请求（最高优先级）
- `m_priority_buffer`：高优先级请求（例如维护类；在 shmem 场景通常为空）
- `m_read_buffer`：普通读请求队列
- `m_write_buffer`：普通写请求队列

队列深度（以代码当前默认值为准）：

- `m_priority_buffer.max_size = 512 * 3 + 32`（在 `setup()` 设置）
- 其余 `ReqBuffer.max_size` 使用 `third_party/ramulator2/src/base/request.h` 的默认值 `32`

read/write 模式切换：

- `wr_low_watermark`（默认 0.2）
- `wr_high_watermark`（默认 0.8）
- 当写队列占用超过高水位或读队列为空时进入 write-mode；当写占用低于低水位且读队列非空时回到 read-mode。

背压来源（上层会看到 enqueue 失败）：

- 上层通过 `Frontend(GEM5)::receive_external_requests()` 把请求送到 `GenericDRAMSystem::send()`；
- `send()` 里 controller 的 `send()` 会尝试把请求入队到 read/write buffer；当 buffer 满时返回 false；上层就会看到 `enq_accepted < enq_attempts`。

> 对上层含义：如果主人看到 shared-memory backend “注入率明显被卡”，优先检查是否需要增大控制器队列容量（v0 目前仍是默认 32），或调整地址映射让访问更分散到不同 layer/bank。

#### 3.0.5 shared memory 请求粒度（v1 的一个重要简化）

当前 shmem backend 往 Ramulator2 送请求时：

- 只传递 `addr` 与 `Read/Write` 类型；
- `shmem_req_t.size` 在 Ramulator2 侧不参与建模（等价于每条请求“固定粒度”的读/写）。

因此 v1 适合用于：

- 对比不同 `P`、不同地址映射、不同 org/timing 对 row conflict/bank conflict 的相对影响；
- 而不适合直接当成“精确带宽模型”（因为未按 size 做 burst/分段/总线占用建模）。

### 3.1 DRAM 组织（org）与地址映射（AddrMapper）

#### 3.1.1 org 的意义

Ramulator2 的 DRAM 组织（`DRAM.org`）决定：

- 有哪些层级（levels），例如 `channel/rank/bank/row/col` 等；
- 每个层级的规模（决定 addr_vec 每个维度的取值范围）；
- addr mapper 需要从地址中切出多少比特给各 level。

在“方案一”中：

- `org.channel` 可以理解为 layer 数（`layer_count`）；
- `org.bank`（或 bank 对应的 level）则决定每个 layer 的 bank 数；
- row/col 则决定 row buffer 行的组织粒度。

#### 3.1.2 AddrMapper 的意义

`GenericDRAMSystem::send()` 会先调用 `AddrMapper::apply(req)`，把 `req.addr` 映射为 `req.addr_vec`，并使用：

- `channel_id = req.addr_vec[0]`

来选择对应的 controller（也就是对应的 layer/channel）。

因此：

- 若你希望“同一 SM 的不同 shared-memory 行/块落在不同 layer 或不同 bank”，需要在 AddrMapper（以及 org 的比特分配）上设计地址位分布；
- 示例里使用 `AddrMapper.impl: RoBaRaCoCh`，是一个线性切片映射（把低位按各 level 位宽依次切出去）。

> 若未来需要更强的可控性（例如显式指定 layer/bank/row/col），可以考虑：
> - 上层构造特定的 `req.addr` 编码；
> - 或新增一个更贴合 shmem 地址语义的 AddrMapper（例如把某些固定字段映射为 channel/bank）。

### 3.2 BankParallel 控制器：端口并行与 bank conflict 约束

`Controller.impl: BankParallel` 的核心行为：

- 每个 controller cycle 最多 issue `P = bank_parallel_ports_per_layer` 次命令；
- 对于 `m_command_meta(cmd).is_accessing == true` 的访问类命令（通常对应 RD/WR）：
  - **同一 cycle 内禁止向同一 bank 并行 issue 多个访问类命令**；
  - bank 的判定依据是地址向量 `addr_vec` 从 `channel` 到 `bank`（包含 bank）的组合键。

这使得：

- 当请求集中打到同一个 bank，会产生“端口/issue 受限 + row buffer 竞争”的综合效果；
- 当请求分散到不同 bank，则能体现 bank 级并行提升（上限受 `P` 限制）。

#### 3.2.1 关键参数

- `Controller.bank_parallel_ports_per_layer`（uint32，默认 1）
  - 解释：每个 layer（每个 channel）每周期可并行推进的“issue 槽位”数量；
  - 用途：把“每 layer 的可并行 bank 数/端口数”抽象成可调参数。

#### 3.2.2 建模选择（当前版本）

当前实现遵循我们之前对齐的选择：

- **仅对访问类（`is_accessing`）命令做 bank 并行约束**；
- ACT/PRE 等开关行类命令不计入 bank 并行端口占用（因此同周期可能出现 ACT 与 RD/WR 并行推进的情况）。

> 这是一种有意的简化：更贴近“我们只关心 row conflict / bank conflict”的第一阶段目标，但在某些极端时序下可能偏乐观。

### 3.3 Row buffer 统计指标（用于 row conflict 分析）

BankParallel 控制器会输出（按 channel 维度）：

- `row_hits_<ch>` / `row_misses_<ch>` / `row_conflicts_<ch>`
- `read_row_*_<ch>`、`write_row_*_<ch>`
- `num_read_reqs_<ch>` / `num_write_reqs_<ch>`
- `avg_read_latency_<ch>`（注意：是 Ramulator2 时钟域内的平均读延迟统计）
- 队列长度统计：`queue_len_avg_<ch>` 等

这些统计可用于：

- 评估 row buffer hit rate（命中/失配/冲突比例）；
- 评估 bank 冲突是否导致排队加剧（队列平均长度、接受率下降等）。

## 4. YAML 配置模板（上层对接用）

下面给出一个“方案一”的最小模板（字段名与位置与示例 YAML 保持一致）：

```yaml
MemorySystem:
  impl: GenericDRAM
  clock_ratio: 1

  DRAM:
    impl: Mono3D
    org:
      channel_width: 256
      prefetch_size: 1
      channel: <layer_count>   # 方案一：layer -> channel
      rank: 1
      bankgroup: 1
      bank: <bank_per_layer>
      row: <rows_per_bank>
      column: <cols_per_row>
      dq: <data_bits>
    timing:
      preset: Mono3D_Default    # 或者按 cycles/ns 覆盖 Mono3D timing 参数集

  Controller:
    impl: BankParallel
    bank_parallel_ports_per_layer: <P>
    Scheduler:
      impl: FRFCFS
    RefreshManager:
      impl: None
    RowPolicy:
      impl: ClosedRowPolicy
      cap: 4
    plugins:

  AddrMapper:
    impl: RoBaRaCoCh
```

> 注意：`RefreshManager.impl` 必须使用 Ramulator2 已注册实现名，例如 `None`（禁用刷新）。示例文件已设置为 `None`。

## 5. 构建与运行要点（Accel-Sim 上层）

### 5.1 构建时的关键宏/依赖

Accel-Sim shared-memory backend 的 real-mode 需要：

- `ENABLE_RAMULATOR`
- `HAVE_RAMULATOR2_HEADERS`（指向 `third_party/ramulator2/src`）
- `HAVE_YAMLCPP_HEADERS`（yaml-cpp include 目录）
- 链接 `third_party/ramulator2/libramulator.so`

（本仓库的 Makefile 已支持通过 `make -C gpu-simulator ENABLE_RAMULATOR=1 ...` 传入 include/lib 路径）

### 5.2 运行时调试：确认 real-mode 是否启用

当 YAML 正确、库可加载时，首次启用会打印类似：

```
[shmem_ramu_dbg] ShmemBackendRamulator2 real mode enabled: clk_ratio=...
```

如未看到该行，说明可能回退到了 placeholder（固定延迟队列）。

### 5.3 导出 Ramulator2 内部统计（强烈建议）

可通过环境变量导出每个 backend 实例的 YAML stats（便于解析 row_hits/row_conflicts 等）：

- `export SHMEM_RAMULATOR2_DUMP_DIR="/tmp/shmem_ramu_dump"`

导出的文件形如：

- `"/tmp/shmem_ramu_dump/shmem_ramulator2_stats_<tag>.yaml"`

（tag 由上层 label 决定，未设置时为 `unknown`）

## 6. 常见坑位与排查清单

- YAML 里 `RefreshManager.impl` 写成未注册的名字（例如 `NoRefresh`）会导致创建失败，进而回退 placeholder。
- `org.channel` 与 `AddrMapper` 的比特切分不匹配会导致 channel 分布异常（例如所有请求都落到 channel 0）。
- 观测到 `enq_accepted << enq_attempts`：
  - 可能是 controller 队列容量太小（ReqBuffer.max_size）或请求注入过快；
  - 也可能是 bank/row 冲突导致队列长期占满。
- CUDA 版本：本项目建议 CUDA 12.x；CUDA 13.x 在某些环境会导致上层 gpgpu-sim 的 `libcuda` shim 编译失败（与本模型无关）。

## 7. 下一步：真正的 Mono3D 设备模型（待时序参数）
当前已经实现 `DRAM.impl: Mono3D`（v1），后续工作主要是把主人提供的 Mono3D **组织/时序参数**补齐到：

- `third_party/ramulator2/src/dram/impl/Mono3D.cpp` 的 `org_presets` / `timing_presets`（可选）
- 或者在 Accel-Sim 的 YAML 中直接用 `org.*` / `timing.n*` 覆盖（推荐先走 YAML，迭代快）

这样 BankParallel 的 bank/row 语义与统计就能在 Mono3D 时序下工作，上层调用接口不需要改动。

## 8. 参数小词典（给上层 Accel-Sim 做处理用）

- `layer_count`：3D 堆叠的 layer 数（v0 中映射为 `DRAM.org.channel`）。
- `P = bank_parallel_ports_per_layer`：每个 layer 的 bank 并行端口数/issue 宽度抽象；v0 中每个 channel/controller 每周期最多 issue P 条命令，并且访问类命令同周期不能落在同一 bank。
- `bank_count`：每个 layer 的 bank 数（由所选 `DRAM.org` preset 决定，或未来 Mono3D 的 org 显式参数决定）。
- `row buffer`：由 DRAM 设备模型维护的 bank 内行状态；`row_hits/row_conflicts/row_misses` 统计用它判定。

### 8.1 Mono3D v1 的 org/timing 可配参数（YAML）

`DRAM.org`（建议显式给出，避免误解 preset）：

- `channel_width`：bit，影响地址映射的 transaction 粒度（tx_bytes = prefetch_size * channel_width / 8）
- `prefetch_size`：列预取倍数（影响 tx_bytes 与 column 有效位数）
- `channel`：layer 数（方案一）
- `rank`：建议固定 1（为兼容 rowpolicy/addrmapper）
- `bankgroup`：建议固定 1（为简化）；需要时可扩展
- `bank`：每 layer 的 bank 数
- `row`：每 bank 行数
- `column`：每行列数（注意与 prefetch_size 的关系）
- `dq`：数据位宽（用于 density 计算）

`DRAM.timing`（可用 preset 或逐项覆盖）：

- `preset: Mono3D_Default`：仅用于跑通/占位（建议主人尽快覆盖）
- 支持用 cycles 直接覆盖：`nBL/nCL/nRCD/nRP/nRAS/nRC/nWR/nRTP/nCWL/nWTR/nRTW/nCCDS/nRRDS/nFAW`
- 可选：提供 `rate` 或 `tCK_ps` 以便用 `tRCD` 这类 ns 参数换算（当前实现对 `t*` 覆盖也支持）
