# FPGA_imu — FPGA 读取 MPU6050 并解算姿态 + 位移（PGL22G 开发板）

用 FPGA（Verilog）通过 I2C 读取 MPU6050 的三轴加速度与角速度，用 **Madgwick 四元数
梯度下降**解算出欧拉角（roll / pitch / yaw，全范围无万向锁），并估计世界系位移
（x / y / z），通过板载 USB 转串口以 **VOFA+ FireWater 协议**输出。

```
FPGA_imu/
├── rtl/
│   ├── i2c_master.v       通用 I2C 主机控制器（可复用，与芯片无关）
│   ├── mpu6050_driver.v   MPU6050 初始化 + 500Hz 周期采集（例化 i2c_master）
│   ├── cordic.v           16级流水线 CORDIC 向量模式（atan2 + 开方）
│   ├── inv_sqrt.v         定点倒数平方根 1/√x（LUT + 牛顿，时分复用单乘法器）
│   ├── madgwick_ahrs.v    Madgwick 姿态解算 + 世界系线性加速度（时分复用单乘法器，例化 3 cordic + 1 inv_sqrt）
│   ├── displacement_calc.v 世界系加速度双重积分 -> 位移（含零速修正）
│   ├── uart_tx.v          UART 发送器（115200 8N1）
│   ├── mpu6050_top.v      顶层：例化全部模块 + Firewater 帧格式化
│   └── attitude_calc.v    （旧）互补滤波方案，已弃用（保留参考）
├── docs/
│   └── 姿态算法说明.md    姿态算法 + 位移 + Firewater 协议详解
├── pds/
│   ├── pgl22g.adc         物理约束：引脚位置/电平标准（PDS）
│   └── pgl22g.fdc         时序约束：50MHz 时钟（PDS）
└── README.md
```

模块依赖关系（低耦合，逐层封装）：

```
mpu6050_top
 ├── mpu6050_driver ── i2c_master
 ├── madgwick_ahrs ── inv_sqrt ×1、cordic ×3（内部单乘法器时分复用）
 ├── displacement_calc
 └── uart_tx
```

## 1. 功能

- 上电自动初始化：延时 100ms → WHO_AM_I(0x75) 校验 → 唤醒(0x6B=0x00) → 稳定 50ms → 配置寄存器
- 每 2ms 突发读 14 字节（0x3B~0x48），输出原始数据 + 帧同步脉冲（500Hz）
- Madgwick 四元数梯度下降解算欧拉角 roll/pitch/yaw（°，全范围无万向锁），全部 Q16.16 定点
- 世界系线性加速度双重积分，估计位移 x/y/z（m，演示级，含零速修正）
- 串口以 VOFA+ FireWater 协议 100Hz 输出 6 通道（roll,pitch,yaw,x,y,z）
- 4 个 LED 指示系统状态（错误/心跳/初始化完成/首帧成功）

## 2. 量程与配置

| 项目 | 配置 | 寄存器值 |
|---|---|---|
| 采样率 | 500 Hz（SMPLRT_DIV=1） | 0x19 = 0x01 |
| 低通滤波 | DLPF_CFG=1（加速度带宽 184Hz / 陀螺 188Hz） | 0x1A = 0x01 |
| 陀螺仪量程 | ±250 °/s，131 LSB/(°/s) | 0x1B = 0x00 |
| 加速度量程 | ±2g，16384 LSB/g | 0x1C = 0x00 |

## 3. 姿态算法

采用 **Madgwick AHRS**（四元数梯度下降，6 轴 IMU），详见 [`docs/姿态算法说明.md`](docs/姿态算法说明.md)：

- 四元数 `q=[q0,q1,q2,q3]` 表示姿态，陀螺仪积分 + 加速度计梯度下降修正漂移：
  - `q̇ = ½·q⊗ω − β·∇f/|∇f|`，`β = 0.1 rad/s`
- 欧拉角（全范围无万向锁）：
  - `roll  = atan2( 2(q0q1+q2q3), 1−2(q1²+q2²) )`
  - `pitch = asin( 2(q0q2−q1q3) )`
  - `yaw   = atan2( 2(q0q3+q1q2), 1−2(q2²+q3²) )`
- 位移：世界系线性加速度双重积分（去重力 + 零速修正 ZUPT）
  > ⚠️ MPU6050 没有磁力计，yaw 无绝对参考，仅积分陀螺仪，会随时间缓慢漂移；
  > 位移为演示级精度，无外部参考会漂移。

## 4. PDS 工程搭建（PGL22G 开发板）

1. 打开 PDS（Pango Design Suite），新建工程：
   - 器件选 **PGL22G**，封装 **MBG324**（手册 2-2 节：FPGA 型号 PGL22G-6CMBG324）
2. 添加设计文件：`rtl/` 下全部 6 个 .v 文件
3. 添加约束文件：`pds/pgl22g.adc`（物理约束）与 `pds/pgl22g.fdc`（时序约束）
4. 综合 → 布局布线 → 生成位流 → 下载（板载 JTAG，手册 3-10 节）
5. 若报 "IO is not enough" 之类的错误，确认：
   - 顶层设为 `mpu6050_top`
   - 未定义 `DEBUG_PORTS` 宏（见第 9 节，宏打开后端口数从 9 变 304，会超出 240 IO 上限）

## 5. PGL22G 引脚连接

### 5.1 板载资源（已固定，约束文件已配好）

| 信号 | 顶层端口 | FPGA 引脚 | 说明 |
|---|---|---|---|
| 50MHz 晶振 | `clk` | B5 | 板载，手册 2-3 节 |
| 复位按键 | `rst_n` | F10 | 板载，低有效，手册 2-7 节 |
| USB 转串口 | `uart_tx` | C10 | 板载 CP2102，手册 3-6 节 |
| LED1~LED4 | `led[3:0]` | U10/V10/U11/V11 | 板载，手册 3-13 节 |

### 5.2 MPU6050（GY-521 模块）接线 —— J8 排针

`i2c_scl`/`i2c_sda` 约束在 J8 排针第 35/36 脚，紧邻电源脚，接线最方便：

| GY-521 模块 | J8 排针 | FPGA 引脚 |
|---|---|---|
| VCC | 第 39 或 40 脚（+3.3V） | — |
| GND | 第 37 或 38 脚（GND） | — |
| SDA | 第 35 脚 | R18（`i2c_sda`） |
| SCL | 第 36 脚 | N14（`i2c_scl`） |

- 板载 I2C 总线（A15/B15）只连了 EEPROM、未引出排针，所以 MPU6050 接 J8 扩展口
- I2C 是开漏总线，**必须外接上拉电阻**；GY-521 模块自带 4.7kΩ 上拉，直接用即可。
  若用自制/裸片模块，需在 SDA、SCL 各接 4.7kΩ 电阻到 3.3V
- 换其他引脚接线时，只需修改 `pds/pgl22g.adc` 中 `i2c_scl`/`i2c_sda` 的
  `PAP_IO_LOC` 两行（J8 全部引脚定义见手册 3-9-2 节）

## 6. LED 状态指示

板载 LED 为低电平点亮，RTL 已按此极性适配，下表"点亮/熄灭"即肉眼所见状态：

| LED | 含义 |
|---|---|
| LED1 (U10) | 错误指示：点亮 = WHO_AM_I 校验失败或 I2C NACK |
| LED2 (V10) | 数据心跳：约 2Hz 闪烁 = 数据正常流动 |
| LED3 (U11) | 初始化完成：进入采样状态后常亮 |
| LED4 (V11) | 首帧成功：收到第一帧有效数据后常亮 |

上电正常时序：LED1 灭 → 约 0.2s 后 LED3 亮 → LED4 亮 → LED2 开始闪烁。

## 7. 串口输出格式（VOFA+ FireWater）

板载 CP2102 的 USB 口直连电脑（需装 CP210x 驱动）。数据按 **VOFA+ FireWater 协议**
输出（逗号分隔 + 换行），100Hz，每帧 54 字节：

```
+029.998,+000.000,+000.000,+000.125,-000.500,+000.000
```

| 通道 | 含义 | 单位 |
|---|---|---|
| 1 | roll | 度 |
| 2 | pitch | 度 |
| 3 | yaw | 度 |
| 4 | 位移 x | 米 |
| 5 | 位移 y | 米 |
| 6 | 位移 z | 米 |

每通道 8 字符（符号 + 3 位整数 + 小数点 + 3 位小数）。上位机用 **VOFA+**
（<https://www.vofa.plus>），协议引擎选 **FireWater**，波特率 115200。

## 8. 上板验证步骤

1. 按第 4 节建好 PDS 工程，按第 5 节接好 MPU6050，下载位流
2. 打开 VOFA+（协议引擎 FireWater，115200），按一下复位键：
   - 静止平放：roll/pitch/yaw ≈ 0，位移 x/y/z 保持 ≈ 0
   - 绕 X 轴翻转：roll 跟随变化（±180°，无万向锁）
   - 绕 Y 轴翻转：pitch 跟随变化（±90°）
   - 绕 Z 轴旋转：yaw 变化（静止后缓慢漂移属正常）
   - 沿某轴快速平移：对应位移通道短暂变化（松手后回落，演示级精度）
3. 若串口无输出，对照第 6 节看 LED：
   - LED1 亮（错误）：检查 SDA/SCL 是否接反、模块供电是否 3.3V
   - LED3 灭：初始化未完成，检查 I2C 接线
   - LED2 不闪：无数据，同上

### 8.1 串口乱码排查

若串口助手收到乱码、且每行长度不一（正常时每帧固定 54 字节），先用板载 4 个 LED
判断 FPGA 内部是否在正常工作，再检查信号链路：

**第一步：看 LED，判断 FPGA 内部状态（最省事的分水岭）**

| LED 现象 | 结论 | 下一步 |
|---|---|---|
| LED3、LED4 亮，LED2 约 2Hz 闪烁 | 初始化完成、数据在流动，**FPGA 侧完全正常** | 问题 100% 在外接链路，见第二步 |
| LED3、LED4 亮，LED2 不闪 | 初始化完成但无数据（可能 I2C 有瞬时错误） | 见 LED1 是否亮：亮=I2C 接线问题 |
| 全部 LED 不亮 | **设计没有跑起来** | 查 `clk`：电平标准必须 LVCMOS33/3.3V（晶振是 3.3V 输出），若 PDS 器件配置里 Bank L0 电压被设成 1.2V 请改回 3.3V；查 `rst_n` 电平（松开复位键后应为高） |
| LED1 常亮 | WHO_AM_I 校验失败（SDA/SCL 接反、MPU6050 没接或供电不对） | 不影响串口：此时串口仍会打印全 0 的数据帧，若串口连全 0 帧都没有，问题仍在串口链路 |

**第二步：外接 USB-TTL 模块链路自查（按概率排序）**

1. **共地**：模块 GND 必须接 J8 的 GND（第 1 或 37/38 脚）。没有公共地，信号没有参考，
   表现正是"乱码 + 长度不一致"——这是外接模块最常见的坑
2. **交叉接线**：FPGA 的 `uart_tx`（J8-3 = T11）是发送端，必须接模块丝印 **RXD** 脚
   （FPGA TX → 模块 RX，交叉接法）。接到模块的 TXD 脚上就收不到正确数据
3. **模块自环测试**：把模块的 TXD、RXD 两根线短接，在串口助手发任意数据，
   能原样回显 = 模块、驱动、USB 线均正常；不能回显 = 模块/驱动问题
4. 换杜邦线、重新插紧（接触不良极常见）
5. 模块供电：3.3V 模块不要接 J8 的 +5V；5V 模块供电正常但注意别把 TXD 接回 FPGA
6. 若以上都正常，用示波器/逻辑分析仪量 T11：应能看到 3.3V 方波，
   位宽约 86.8µs（115200bps），每 10ms 一帧突发（100Hz）

**板载 CP2102 串口（不接外接模块时）**：`uart_tx` 必须约束到 C10（板载 CP2102 的
RXD 输入脚，手册 3-6-2 表）。约束到其他引脚时 C10 悬空 → CP2102 把噪声当数据发给
电脑，表现同样为乱码。另外确认插的是 USB-UART 口（CP2102）而非 USB2.0 口（FT232H），
VOFA+ 数据引擎选"FireWater"（不要选 JustFloat 等私有协议），编码 UTF-8/GBK 均可。

## 9. 二次开发接口

### 9.1 调试观察

本版顶层精简为 **9 个 IO**（`clk/rst_n/i2c_scl/i2c_sda/uart_tx/led[3:0]`），
未再引出宽调试端口。需观察内部信号（四元数、世界加速度、位移、状态机等）时，
推荐用 **PDS 在线逻辑分析仪（DebugCore）**，无需占用 IO。
`madgwick_ahrs.v` 额外保留了 `q0_dbg~q3_dbg` 四元数调试输出（顶层未接），
可在仿真或 DebugCore 中观测。Q16.16 换算：实际值 = 值(有符号) ÷ 65536。

### 9.2 模块级复用

- `i2c_master.v`：通用 I2C 主机，与 MPU6050 无关，可直接复用到任何 I2C 芯片
- `uart_tx.v`：通用串口发送器，`tx_start` 脉冲触发、`tx_busy` 握手
- `cordic.v`：纯流水线 atan2/开方（度输出），可复用
- `inv_sqrt.v`：定点倒数平方根，可复用
- `madgwick_ahrs.v` / `displacement_calc.v`：姿态/位移解算核心（独立模块，可替换）

## 10. 已知限制

- yaw 漂移：无磁力计，仅积分陀螺仪 z 轴，长时间会漂移
- 位移漂移：双重积分无绝对参考，ZUPT 对水平匀速运动不敏感，位移仅演示级精度
- 加速度计持续大加速度时姿态收敛变慢（梯度下降以重力为参考）

## 11. 参考

- Madgwick, S. O. H. "An efficient orientation filter for inertial and inertial/magnetic sensor arrays", 2011
- x-io Technologies `MadgwickAHRS.c`（开源 C 参考实现）
- VOFA+ FireWater 协议：<https://www.vofa.plus/docs/learning/dataengines/firewater/>
- MPU-6000/MPU-6050 Product Specification，Rev 3.4（本目录 PDF）
- MPU-6000/MPU-6050 Register Map and Descriptions，RM-MPU-6000A-00，Rev 4.2
- PGL22G 开发板用户手册 Rev1.1（ALINX，板卡引脚定义）
- 姿态/位移/定点算法详解：`docs/姿态算法说明.md`
