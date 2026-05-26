# STM32F103 无人船/无人车项目分析报告

## 📊 项目概述

**项目名称**：UAS-Project-STM32（无人船/无人车系统）  
**作者**：matreshka15  
**联系方式**：8523429@qq.com  
**GitHub**：https://github.com/matreshka15/USV-STM32F103-part  

**项目功能**：
- 无人船/无人车下位机控制程序
- 配合树莓派、Jetson Nano等上位机使用
- 支持手动遥控、GPS自主导航、路径规划

---

## 🔧 硬件配置

### 主控芯片
- **型号**：STM32F103ZET6（大容量产品）
- **主频**：72MHz
- **原因**：使用了定时器8，必须使用大容量产品

### 传感器配置
| 传感器 | 型号 | 功能 | 接口 |
|--------|------|------|------|
| GPS模块 | UBlox M8 | 定位、导航 | USART3 + DMA |
| 姿态传感器 | MPU6050 | 加速度计、陀螺仪 | I2C |
| 磁力计 | HMC5883（可选） | 电子罗盘 | I2C |
| 遥控器接收机 | WFR07/WFT07 | 手动控制 | PWM输入捕获 |

### 执行机构
| 执行器 | 型号 | 控制方式 |
|--------|------|----------|
| 前轮转向 | 舵机 | PWM |
| 后轮电机 | 有刷电机 x2 | PID调速 |

### 硬件连接
参考文件：`STM32资源使用情况.xlsx`、`WFR07.jpg`、`WFT07.jpg`、`舵机线.png`

---

## 🏗️ 软件架构

### 1. 系统框架设计

#### 类操作系统式任务调度
项目采用**类操作系统式框架**，任务按不同频率被调用：

```
系统节拍：5ms（TIM5定时中断）

任务频率分配：
- 200Hz（5ms）：最高优先级任务
- 100Hz（10ms）：高频任务
- 50Hz（20ms）：姿态解算
- 10Hz（100ms）：传感器数据读取
- 5Hz（200ms）：与上位机通信
- 1Hz（1000ms）：系统状态监控、喂狗
```

**优点**：
- 提高系统灵活性
- 降低系统耦合性
- 类似无人机飞控厂商（如Crazyflie）的设计

#### 任务调度实现
**文件**：`User/LoopSequence/Loop.c`

```c
void Loop(void)
{
    /*--------1Hz级任务----------*/
    if(Mission.Hz1) {
        feed_dog();  // 喂看门狗
        // 状态机处理
        // LED闪烁指示
    }
    
    /*--------5Hz级任务----------*/
    if(Mission.Hz5) {
        synthesis_frame();  // 向上位机发送姿态数据
    }
    
    /*--------10Hz级任务---------*/
    if(Mission.Hz10) {
        TIM3_4channel_process();  // 遥控器信号捕获
        Analyze_Controller_Msg();  // 分析遥控器消息，切换状态
        pvtFromGPS();             // GPS数据解析
        receiveComData();          // 读取MPU6050数据
        dataFrom6050();
        filter_IMU_data(10);     // 滤波
    }
    
    /*--------50Hz级任务---------*/
    if(Mission.Hz50) {
        // Madgwick梯度下降姿态解算
        MadgwickAHRSupdate(...);
        Convert_Quaternion_To_Euler();  // 四元数转欧拉角
    }
}
```

**定时器中断服务函数**：
```c
void TIM5_IRQHandler(void)
{
    system_pulse++;
    
    if((system_pulse+1) % (1) == 0)    Mission.Hz200 = 1;
    if((system_pulse+1) % (2) == 0)    Mission.Hz100 = 1;
    if((system_pulse+1) % (4) == 0)    Mission.Hz50  = 1;
    if((system_pulse+1) % (20) == 0)   Mission.Hz10  = 1;
    if((system_pulse+1) % (40) == 0)   Mission.Hz5   = 1;
    if((system_pulse+1) % (200) == 0)  Mission.Hz1   = 1;
    
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}
```

---

### 2. 有限状态机设计

**文件**：`CONFIGURATION.h`

```c
/* 下位机有限状态机 */
#define STATE_INITIALIZE        0  // 初始化状态
#define STATE_NORMALLY_RUNNING 1  // 程序正常运行（自主导航）
#define STATE_COLLECTING_POINTS 2  // GPS采点状态
#define STATE_MANUAL_CTRL      3  // 手动遥控状态
#define STATE_DEBUGGING       4  // 程序测试状态
```

**状态切换逻辑**（通过遥控器切换）：
- 通道0 <= 1200：GPS采点状态
- 1200 < 通道0 <= 1800：自主导航状态
- 通道0 > 1800：手动遥控状态

**文件**：`User/LoopSequence/Loop.c` 的 `Analyze_Controller_Msg()` 函数

---

### 3. 传感器数据融合

#### Madgwick梯度下降姿态解算算法

**为什么选择Madgwick？**
1. **低性能要求**：仅需10Hz采样速度即可收敛
2. **弥补卡尔曼滤波缺点**：卡尔曼滤波吃性能
3. **适合STM32F1**：72MHz主频无法同时运行姿态解算和控制

**算法原理**：
- 梯度下降算法在机器学习中广泛应用
- 使函数输出值与理论值之间误差逐步下降
- 在姿态解算上：使解算姿态与真实姿态之间误差逐步下降

**文件**：`User/DATAFUSION/DATAFUSION.c`

```c
void MadgwickAHRSupdate(
    float gx, float gy, float gz,  // 陀螺仪数据
    float ax, float ay, float az,  // 加速度计数据
    float mx, float my, float mz,  // 磁力计数据
    float beta                   // 增益，随时间自动下降
) {
    // 梯度下降算法实现
    // 更新四元数 SEq_1 ~ SEq_4
}
```

**四元数转欧拉角**：
```c
void Convert_Quaternion_To_Euler(void)
{
    // 四元数转欧拉角公式
    yaw   = atan2(2*(SEq_1*SEq_2 + SEq_3*SEq_4), 
                  1 - 2*(SEq_2*SEq_2 + SEq_3*SEq_3));
    pitch = asin(2*(SEq_1*SEq_3 - SEq_4*SEq_2));
    roll  = atan2(2*(SEq_1*SEq_4 + SEq_2*SEq_3), 
                  1 - 2*(SEq_3*SEq_3 + SEq_4*SEq_4));
}
```

---

### 4. 环形缓冲区设计

**目的**：提高数据高速传输时的可靠性，降低丢包率

**文件**：`User/RingBuff/ring_buff.c`

```c
typedef struct {
    volatile u16 Head;
    volatile u16 Tail;
    volatile u16 Length;
    volatile u8 Ring_Buff[LENGTH_OF_BUFF];
} RingBuff_t;

volatile RingBuff_t ringBuff;       // GPS数据缓冲区
volatile RingBuff_t ringBuff_IMU;   // IMU数据缓冲区
volatile RingBuff_t ringBuff_USART1; // 上位机通信缓冲区
```

**使用场景**：
- GPS模块接收大量数据，严重占用MCU任务处理时间 → 使用DMA + 环形缓冲区
- 上位机通信：高速数据传输

---

### 5. GPS模块配置

**型号**：UBlox M8

**协议**：UBX协议（比NMEA协议更高效）

**优化**：
- 系统上电前自动配置GPS输出必要信息
- 滤除不必要信息，大幅提高数据处理效率

**文件**：`User/GPS/GPS.c`

```c
void GPSDATA_Init(void)
{
    UBX_CFG_RATE();      // 配置输出速率
    UBX_CFG_DATAOUT();  // 配置输出信息
    UBX_CFG_MODEL();    // 配置动态模型（车载/海上）
}

#define MODEL_VEHICLE_SEA 0  // 1：海上模型；0：车载模型
```

**可设置的载具模型**：
- 汽车模型：优化GPS输出数据适配陆地移动
- 海上模型：优化GPS输出数据适配海上移动

---

### 6. 电机PID控制

**文件**：`User/PID/PID.c`

```c
#define Velocity_KP (float)12
#define Velocity_KI (float)-3.5
#define Velocity_KD (float)0

void Minimize_Greatest_Error_Increment_PID(
    int16_t current_speed_left,
    int16_t current_speed_right,
    int16_t target_speed
) {
    // 增量式PID算法
    // 使左右轮速度基本相同
}
```

**功能**：
- 使用PID控制两电机转速基本相同
- 增量式PID，避免积分饱和

---

### 7. 路径规划与自主导航

**文件**：`User/PathPlanning/PathPlanning.c`

```c
void Execute_Planned_Path(
    float current_yaw,
    u16 target_yaw,
    u16 target_distance,
    u8 control_status,
    u8 frequency
) {
    // 根据当前姿态和目标航点，计算转向角和速度
    // 实现自主导航
}
```

**功能**：
- 输入GPS坐标，自主导航
- 可在Google Map上标点直接导出为载具轨迹

---

### 8. 上位机通信

**通信方式**：USART1（DMA可选）

**数据帧格式**：
```c
#define one_frame_length 22  // 数据帧长度共22Byte
u8 DATAFRAME[one_frame_length] = {0x73, 0x63};  // 帧头

typedef struct {
    u16 yaw;           // 航向角
    u16 distance;       // 距离
    u8 control_status;  // 控制状态（第7位：EndOfNav标志位）
} Route;
```

**上位机端程序**：
- 非ROS版（Python）：https://github.com/matreshka15/raspberry-pi-USV-program
- ROS版：https://github.com/matreshka15/ROS-based-unmanned-vehicle-project

---

## 📂 文件结构

```
USV-STM32F103-part-master/
├── CONFIGURATION.h              # 配置文件（宏定义、条件编译）
├── README.md                   # 项目说明
├── STM32资源使用情况.xlsx      # 硬件资源分配表
├── WFR07.jpg                  # 遥控器接线图
├── WFT07.jpg                  # 遥控器接线图
├── 舵机线.png                 # 舵机接线图
├── Libraries/                 # 库文件
│   ├── CMSIS/                # Cortex-M3核心支持
│   │   └── CM3/
│   │       ├── CoreSupport/           # 核心支持（core_cm3.c/h）
│   │       └── DeviceSupport/        # 设备支持
│   │           └── ST/STM32F10x/
│   │               ├── stm32f10x.h
│   │               ├── system_stm32f10x.c/h
│   │               └── startup/              # 启动文件（arm/gcc_ride7/iar）
│   └── STM32F10x_StdPeriph_Driver/  # 标准外设库
│       ├── inc/                       # 头文件
│       │   ├── stm32f10x_gpio.h
│       │   ├── stm32f10x_rcc.h
│       │   ├── stm32f10x_usart.h
│       │   ├── stm32f10x_tim.h
│       │   ├── stm32f10x_i2c.h
│       │   ├── stm32f10x_dma.h
│       │   └── ...（其他外设）
│       └── src/                       # 源文件
│           ├── stm32f10x_gpio.c
│           ├── stm32f10x_rcc.c
│           ├── stm32f10x_usart.c
│           ├── stm32f10x_tim.c
│           └── ...（其他外设）
├── Project/                   # 工程文件
│   ├── Template.uvprojx      # Keil5工程文件
│   ├── Template.uvoptx
│   └── ...
├── Output/                   # 输出文件
│   └── UAS.hex             # 可烧录的hex文件
└── User/                     # 用户代码
    ├── Inclusion.h           # 头文件汇总
    ├── main.c               # 主函数
    ├── initialize.c          # 初始化代码
    ├── stm32f10x_conf.h    # 库配置文件
    ├── stm32f10x_it.c/h    # 中断服务函数
    ├── system_stm32f10x.c/h # 系统时钟配置
    ├── COMPASS/             # 电子罗盘模块
    │   ├── COMPASS.c/h
    ├── DATAFUSION/          # 数据融合（姿态解算）
    │   ├── DATAFUSION.c/h
    ├── Delay/               # 延时模块
    │   ├── delay.c/h
    ├── EXT/                 # 外部中断模块
    │   ├── ext.c/h
    ├── FILTER/              # 滤波模块
    │   ├── FILTER.c/h
    ├── GPS/                 # GPS模块
    │   ├── GPS.c/h
    ├── GeneralFunction/      # 通用功能模块
    │   ├── General_function.c/h
    ├── IIC/                 # I2C通信模块
    │   ├── IIC.c/h
    ├── IMU/                 # 惯性测量单元模块
    │   ├── IMU.c/h
    │   └── MPU6050.h
    ├── INPUT_CAPTURE/       # 输入捕获模块（遥控器）
    │   ├── INPUT_CAPTURE.c/h
    ├── LoopSequence/        # 主循环任务调度
    │   ├── Loop.c/h
    ├── Motor/               # 电机控制模块
    │   ├── Motor.c/h
    ├── NRF24L01/           # 无线模块（未使用）
    │   └── NRF24L01.c
    ├── PID/                 # PID控制模块
    │   ├── PID.c/h
    ├── PWM/                 # PWM模块（舵机、电机）
    │   ├── pwm.c/h
    ├── PathPlanning/        # 路径规划模块
    │   ├── PathPlanning.c/h
    ├── RingBuff/            # 环形缓冲区模块
    │   ├── ring_buff.c/h
    ├── TD/                  # 数据传输模块
    │   ├── TD.c/h
    ├── Usart/               # 串口通信模块
    │   ├── USART.c/h
    ├── WDG/                 # 看门狗模块
    │   ├── WDG.c/h
    └── WirelessUSART/       # 无线串口模块
        └── WirelessPort.c/h
```

---

## 🚀 使用方法

### 1. 硬件连接
1. 按照 `STM32资源使用情况.xlsx` 表格连接传感器
2. 参考 `WFR07.jpg`、`WFT07.jpg`、`舵机线.png` 连接遥控器和舵机

### 2. 软件编译与下载
1. 打开Keil5
2. 打开 `Project/Template.uvprojx`
3. 编译（Build）
4. 下载到STM32（Load）

### 3. 运行模式
- **仅下位机**：只能遥控控制
- **连接上位机**：可以进行定点巡航、采点等高级功能

### 4. 上位机程序
- **推荐**：非ROS版（Python）
  - 原因：开发环境容易配置，稳定性好
  - 地址：https://github.com/matreshka15/raspberry-pi-USV-program
  
- **进阶**：ROS版
  - 地址：https://github.com/matreshka15/ROS-based-unmanned-vehicle-project

---

## 🔍 核心亮点

### 1. 系统架构方面
✅ **类操作系统式框架**：任务可分不同频率调用，提高灵活性，降低耦合性  
✅ **条件编译**：只需改动一个宏定义，系统即可自动选择传感器（GY901与MPU6050+HMC5883无缝切换）  
✅ **传感器自动自检、校准**：保障数据精度  

### 2. 系统核心方面
✅ **环形缓冲区**：大幅降低数据高速传输时的丢包率  
✅ **有限状态机**：可通过遥控器进入不同状态，提高灵活性  
✅ **Madgwick梯度下降算法**：仅需10Hz采样速度即可收敛，适合低性能MCU  
✅ **GPS DMA**：严重占用MCU任务处理时间的GPS模块加入DMA支持  

### 3. 功能拓展方面
✅ **可设置的载具模型**：汽车模型、海上模型，优化GPS数据输出  
✅ **Google Map标点**：可直接导出为载具轨迹  
✅ **语音提示**：GPS连接情况、机体运行情况（需接音箱）  

---

## 🛠️ 可能的改进建议

### 1. 性能优化
- **升级到STM32F4系列**：主频168MHz，可以运行更复杂的算法（如EKF）
- **使用硬件FPU**：STM32F4有硬件浮点单元，加速姿态解算

### 2. 功能增强
- **添加超声波/激光雷达**：避障功能
- **添加摄像头**：视觉导航
- **支持更多通信方式**：LoRa、NB-IoT（远程控制）

### 3. 代码优化
- **使用RTOS**：目前是裸机程序，可以添加FreeRTOS/RT-Thread，提高可维护性
- **模块化重构**：部分模块耦合度较高，可以进一步解耦
- **添加单元测试**：提高代码可靠性

### 4. 文档完善
- **添加详细注释**：部分函数缺少注释
- **提供PDF版原理图**：方便硬件调试
- **录制教学视频**：帮助新手快速上手

---

## 📚 学习重点与路径建议

### 初级阶段（0-3个月）
1. **C语言基础**：指针、内存管理、位操作
2. **STM32基础**：GPIO、定时器、中断、USART、I2C
3. **传感器使用**：MPU6050、HMC5883、GPS
4. **简单项目**：LED闪烁、按键输入、串口通信

### 中级阶段（3-6个月）
1. **PID控制**：理解PID原理，实现电机调速
2. **数据融合**：理解互补滤波、卡尔曼滤波、Madgwick算法
3. **通信协议**：理解UBX协议、自定义通信协议
4. **中等项目**：平衡车、循迹车

### 高级阶段（6个月以上）
1. **姿态解算**：深入理解四元数、欧拉角、方向余弦矩阵
2. **路径规划**：理解A*算法、Dijkstra算法
3. **嵌入式Linux**：驱动开发、系统构建
4. **复杂项目**：无人机、无人船、机器人

---

## 🔗 重要资源链接

### 项目相关
- **下位机GitHub**：https://github.com/matreshka15/USV-STM32F103-part
- **上位机（非ROS版）**：https://github.com/matreshka15/raspberry-pi-USV-program
- **上位机（ROS版）**：https://github.com/matreshka15/ROS-based-unmanned-vehicle-project
- **开发日志与资料**：https://github.com/matreshka15/unmanned-ship-datasheets

### 学习资源
- **姿态解算算法解释与实机演示**：https://zhuanlan.zhihu.com/p/82973264
- **Madgwick算法论文**：https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
- **STM32标准外设库手册**：ST官方网站
- **UBlox M8数据手册**：UBlox官方网站

### 开发工具
- **Keil5**：编译、下载、调试
- **STM32CubeMX**：图形化配置工具
- **Serial Port Utility**：串口调试助手
- **uCenter**：UBlox GPS配置工具

---

## 📝 总结

这是一个**设计精良的无人船/无人车下位机程序**，具有以下特点：

✅ **专业的系统架构**：类操作系统式任务调度、有限状态机设计  
✅ **高效的算法选择**：Madgwick梯度下降算法，适合低性能MCU  
✅ **可靠的硬件设计**：环形缓冲区、DMA、看门狗  
✅ **灵活的配置方式**：条件编译，传感器无缝切换  
✅ **完整的功能实现**：手动遥控、GPS自主导航、路径规划  

**适合人群**：
- 嵌入式开发初学者：学习STM32外设使用、传感器数据融合
- 进阶开发者：学习系统架构设计、姿态解算算法
- 无人机/无人船爱好者：参考项目设计，快速搭建自己的无人系统

**下一步建议**：
1. 搭建硬件环境，编译下载程序
2. 理解Madgwick算法原理
3. 尝试修改PID参数，观察系统响应
4. 添加自己的功能（如避障、视觉导航）

---

**报告完成日期**：2026年5月23日  
**分析工具**：WorkBuddy AI助手  
**项目作者**：matreshka15  
**联系方式**：8523429@qq.com
