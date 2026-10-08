# 体感遥控超声波巡检小车

以 STM32F407ZG 为主控的麦克纳姆轮巡检小车原型，结合二维舵机云台、超声波测距、四合一气体检测和 ESP32-CAM 图传。手持 NodeMCU 通过 Wi-Fi 发送体感或摇杆指令，电脑/手机网页显示扫描点、气体读数和编码器估算路线，支持保存路线及绑定位置的扫描快照。

这是嵌入式比赛与学习工程。超声波点图是有限角度、有限分辨率的测距可视化，编码器路线会受打滑与标定误差影响。没有宣称激光雷达级建图、SLAM、矿用认证或可靠自主导航；自动按保存路线行驶功能已在 V8.9 移除。

**新 AI / 新成员接手请先看 [工程与开发对话摘要](docs/AI_HANDOFF.md)**：包含当前架构、阅读顺序、重要历史决定、已解决问题、暂缓建议、用户偏好和可复制的交接提示词。

## 当前版本与入口

| 部分 | 版本 | 正确入口 |
|---|---|---|
| STM32 主控 | V8.13 | `project/Gimbal_STLink.uvprojx`，目标 `Gimbal STLink` |
| 基座 ESP-01S | V4.9 | `project/ESP01S_Base_UDP/ESP01S_Base_UDP.ino` |
| 手持 NodeMCU V3 CH340G | V1.6 | `project/NodeMCU_Handheld_MPU_Car/NodeMCU_Handheld_MPU_Car.ino` |
| AI Thinker ESP32-CAM | V1.2 | `project/ESP32CAM_Camera/ESP32CAM_Camera.ino` |
| 二维网页 | V8.12 | `project/gimbal-radar-standalone.html` |
| 三维网页 | V8.12 | `project/gimbal-3d-scan.html` |

`ESP01S_Handheld_MPU_UDP`、`ESP8266_Firmware`、`ESP32S3_Camera`、`MiniProgram` 和 `web-dashboard` 保留为历史/辅助资料，不是当前整机烧录组合。特别是当前摄像头实物为普通 ESP32-CAM，不要烧录 S3 目录。历史版本说明及 ESP 文件头部分接线尚为旧版，当前接线以本 README 和 `project/User/main.c` 顶部 V8.13 完整接线表为准。

## 已实现功能

- MPU6050 体感控制云台，或切换为体感控制小车。
- ADS1115 读取双轴摇杆；按摇杆切换三种模式，长按请求软件停车。
- 云台自动扫描、HC-SR04 测距、蜂鸣器/语音/灯带距离提示。
- 四合一气体模块通过 Modbus-RTU 提供 CH4、O2、CO、H2S 通道数据及状态。
- 四轮霍尔编码器估算轮速、位姿与行驶路线。
- 2D/3D 网页显示扫描、轮速和气体信息，可查看 ESP32-CAM 画面。
- 路线以本地 JSON 保存；路线上的扫描记录可点击回看。快照保存点击时的现有点缓存，不自动判定一轮扫描是否完成。

## 通信与当前接线

手持 NodeMCU → UDP → 基座 ESP-01S → USART2 → STM32；STM32 将遥测和气体读数回传基座，网页通过 HTTP 获取。摄像头加入同一热点，网页直接向摄像头取图。

| 连接对象 | 模块信号 | STM32 引脚 | 设置 |
|---|---|---|---|
| 基座 ESP-01S | RX | PA2 / USART2_TX | 115200，8N1 |
| 基座 ESP-01S | TX | PA3 / USART2_RX | 115200，8N1 |
| 气体模块 | R/RX | PC10 / UART4_TX | 9600，8N1 |
| 气体模块 | T/TX | PC11 / UART4_RX | 9600，8N1 |

基座 VCC、EN 接稳定 3.3V；气体模块按现有 4MZ-HH4 资料接稳定 5V，串口逻辑为 3.3V；与 STM32 共地。手持 I2C：D1/GPIO5=SCL、D2/GPIO4=SDA，MPU6050、OLED、ADS1115 共用；摇杆 SW 接 D5/GPIO14，X/Y 分别接 ADS1115 A0/A1，供电 3.3V。电机、显示屏、编码器及全部供电细节见 `project/User/main.c`。

## 开发环境与依赖

| 工具/依赖 | 对应配置 |
|---|---|
| Keil MDK 5 | ARMCC 5.06；STM32F4xx_DFP 1.0.8；ST-Link/SWD |
| STM32 标准外设库、启动文件 | 已随 `Library/`、`Start/` 收录，保持原始版权注释 |
| MCU 时钟 | 8 MHz HSE，原代码配置系统时钟；不要根据旧 IDE 显示值改晶振参数 |
| Arduino ESP8266 Core | 3.1.2；基座 Generic ESP8266 Module，手持 NodeMCU 1.0 / ESP-12E Module |
| Arduino ESP32 Core | 3.3.11；AI Thinker ESP32-CAM，DIO、40 MHz、Huge APP、PSRAM |
| JavaScript | 浏览器本地依赖 `three.min.js`、`OrbitControls.umd.js` 和各功能脚本，均已收录 |
| 验证与同步 | Python 3.7+；Node.js 18+；Git；GitHub 已认证凭据 |

本工程手持 OLED、MPU 和 ADS1115 驱动位于手持目录中，使用 Core 自带 Wire；不需要另装 Adafruit 库。依赖版本记录见 [dependencies.json](dependencies.json)。

## 首次编译与运行

1. 克隆整个仓库，保持 `project/` 下各网页脚本相对位置。
2. 每个需要烧录的 ESP 目录中，将 `secrets.example.h` 复制为 `secrets.h`，填入自行设置的 Wi-Fi 密码。当前基座、手持、摄像头必须一致，密码 8–63 个字符。`secrets.h` 已被 Git 忽略；公开仓库不含原项目密码。
3. STM32：用 Keil 打开 `Gimbal_STLink.uvprojx`，选择 `Gimbal STLink`，Rebuild 后通过 ST-Link 下载。V8.13 使用上表串口接线。
4. Arduino IDE：安装指定 ESP8266/ESP32 Core，分别打开对应目录的同名 `.ino`。基座上传速度 115200、CPU 80 MHz、Flash DOUT/40 MHz；手持选 NodeMCU 1.0；摄像头选 AI Thinker ESP32-CAM，按板载下载电路进入烧录模式。详见各固件注释及历史烧录说明中的板卡设置。
5. 基座启动 `GimbalCtrl` 热点，手持和摄像头连接该热点。电脑/手机也连接同一热点；打开 `http://192.168.4.1` 是基座内置模式控制页。
6. 完整 2D/3D 扫描界面使用仓库网页。电脑可直接打开上述 HTML；也可在 `project` 中运行 `python -m http.server 8000 --bind 0.0.0.0`，电脑访问 `http://localhost:8000/gimbal-3d-scan.html`，手机访问 `http://电脑在热点中的IP:8000/gimbal-3d-scan.html`。电脑需允许局域网入站访问。手机必须和提供文件的电脑处于同一网络，不能只传单独 HTML。
7. 网页基座地址默认 `http://192.168.4.1`。摄像头地址从摄像头串口日志确认后填入图传面板；摄像头 IP 可能随 DHCP 变化。首次控制小车先架空核对轮向、回中与失联响应。

路线和扫描档案存储在使用者电脑本地，不依赖 Navicat 或在线数据库。浏览器若不支持目录选择，可使用 JSON 导出/打开；不同浏览器的本地文件权限存在差异。不要通过公开 HTTPS 托管页面直接访问局域网 HTTP 设备来替代上述本地运行方式。

## 本地原工程 → D 盘仓库 → GitHub

原工作工程保持原位置；这个仓库是经过公开脱敏的同步副本。原工程路径、工具路径和同步清单只存放在未跟踪的 `.local/` 中。

- 本地 Git 仓库：`D:\嵌入式Git仓库\gimbal-inspection-rover`。
- 覆盖前的差异备份：`D:\嵌入式Git备份\gimbal-inspection-rover`；只备份同步中实际被替换/删除的已管理文件，首次导入无旧文件时不重复备份。
- 初次在另一台电脑配置：运行 `tools/setup.ps1` 选择原工程目录，必要时在 `.local/config.json` 填入 `node`、`keil`、`arduino_cli` 工具路径；设置 `arduino_build_root` 为 D 盘纯英文路径。
- 日常双击 **`一键同步备份.cmd`**，输入具体变更说明。脚本按顺序同步、脱敏扫描、网页验证、已配置的 Keil/三个 ESP 固件编译、Git 提交、推送并比较远端提交号。本机已配置这些编译工具；其他电脑缺少配置时会明确显示相应编译跳过。
- ESP32 工具链在 Windows 的中文输出目录可能出现链接失败；本机统一在 `D:\EmbeddedBuildCache\gimbal-inspection-rover` 构建，不移动原工程。Arduino IDE 用户可使用工具支持的英文构建目录。
- 无变更时不制造空提交，但仍会重试推送已有本地提交；推送失败时保留本地版本，再运行即可。
- 脚本仅从原目录读取，保留相对目录；若发现 D 盘受控副本被单独编辑，会停止覆盖并提示先合并。源文件在同步期间发生变动也会报错。
- 原 Wi-Fi 密码在导出时改为 `#include "secrets.h"` 和宏引用，仅影响 D 盘发布副本。首次同步为本机生成被忽略的私有头文件；以后不会覆盖你修改过的私有配置。密码变更时手工同步各 ESP 的 `secrets.h`。
- `.git` 旧历史、编译文件、日志、压缩包、报告截图、IDE 缓存、个人配置、实际路线档案和密钥不会上传。微信小程序 AppID 使用公开测试占位值。

命令行方式：

```powershell
python -B tools/project_sync.py sync
python -B tools/project_sync.py validate
python -B tools/project_sync.py publish --message "fix: 描述本次已验证的具体修改"
```

网页现有快照测试也可单独执行：`node project/tests/route-snapshot.test.cjs`。

## 验证边界和版权

源码检查/编译不等于整车已完成实测。每次更改后的真实接线、方向、运动、图传和传感器表现需要设备验证。发布记录见 [docs/publication.md](docs/publication.md)。

第三方代码沿用原许可证及版权，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本次未替作者选择全仓库开源许可证；公开可见不代表对所有原创代码授予任意商用或再分发许可。
