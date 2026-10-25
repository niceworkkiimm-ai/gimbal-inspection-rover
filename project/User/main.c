/**
 * 项目迭代: V8.13（适配PCB：基座与气体模块串口用途互换）
 * 当前版本: 网页V8.12；STM32 V8.13、基座ESP V4.9、手持V1.6、ESP32CAM_Camera V1.3
 * 最近更新: 2026-10-25（仅摄像头画面方向；STM32执行代码未变）
 * 摄像头V1.3：相对V1.2将画面旋转180度，只需重烧ESP32CAM_Camera。
 *   - CAMERA_VFLIP=0、CAMERA_HMIRROR=1；帧率、分辨率和完整接线不变。
 * 以下为STM32 V8.13（2026-10-02）及更早版本的历史变更。
 * V8.13：基座ESP使用USART2(PA2 TX/PA3 RX)，115200/8N1。
 *   - ESP RX->PA2(USART2_TX)，ESP TX->PA3(USART2_RX)。
 *   - 气体模块使用UART4(PC10 TX/PC11 RX)，9600/8N1，Modbus-RTU。
 *   - 模块R/RX->PC10(UART4_TX)，模块T/TX->PC11(UART4_RX)。
 *   - GPIO复用、APB1外设时钟、收发寄存器、NVIC及IRQ入口同步交换。
 *   - 中断优先级仍按用途设置：无线1/0、气体1/1；队列、超时和通信协议不变。
 *   - 只需重烧STM32；基座ESP、手持、摄像头和网页无需重新烧录/替换。
 *   - 完整接线表已更新；PC10/PC11只用于气体UART4，不同时连接USART3/SD卡。
 * 以下V8.12及更早条目为历史记录；串口用途和接线以V8.13及下方完整接线表为准。
 * V8.12：实时三维页与扫描快照回看共用gimbal-model.js，统一为原版详细外壳/云台模型。
 *   - 移除实时射线旁的半透明ConeGeometry大三角形，保留细射线与测距点。
 *   - 路线快照JSON version 2格式不变；已有快照用新版网页打开即采用详细模型，无需重新保存。
 *   - 网页文件、scan-snapshot.js与新增gimbal-model.js需一起替换并刷新。
 *   - 本main.c仅更新顶部注释；执行代码与FIRMWARE_VERSION仍为V8.10，所有设备无需重烧。
 *   - 完整接线、供电、轮向和控制协议均不变；基座仍为UART4 PC10 TX/PC11 RX、115200/8N1。
 *   - 详见V8.12_三维模型统一与射线显示.md；以下既有版本历史完整保留。
 * V8.11：2D/3D网页在路线位置保存当前扫描点图，红点可点击回看，3D回看可旋转。
 *   - 保存路线JSON version 2包含扫描快照；打开后恢复红点和点图，仍支持旧version 1路线。
 *   - 快照保存当前显示缓存，不代表一次完整扫描自动完成；路线仍为编码器估算位置。
 *   - 保存当前扫描点要求在线、新鲜路线/扫描数据及原生路线记录同步完成。
 *   - 最多100个点位，每点最多800个采样；路线JSON文件上限20MB，CSV不包含扫描快照。
 *   - 不存档气体/图传，不恢复自动行驶；完整接线、电机/编码器/摇杆方向及供电保持原样。
 *   - 接线复核：基座UART4仍为PC10 TX、PC11 RX、115200/8N1，其余以下完整接线表不变。
 *   - 只需替换并刷新网页；所有设备均无需重新烧录，无需重新标定实车方向。
 *   - 本main.c仅补充注释，执行代码与FIRMWARE_VERSION仍为V8.10；详见V8.11使用说明。
 * V8.10：基座ESP通信由USART1(PA9/PA10)迁移UART4(PC10/PC11)，115200/8N1不变。
 *   - ESP TX->PC11(UART4_RX)，ESP RX->PC10(UART4_TX)；撤掉原PA9/PA10的ESP信号线。
 *   - GPIOC/AF8、APB1 UART4时钟、收发寄存器与UART4_IRQHandler同步修改。
 *   - 只需重烧STM32；基座/手持/摄像头固件、网页和通信协议不变。
 *   - 下方完整接线已更新；TFT、气体USART2、电机、编码器及所有供电保持原样。
 *   - PC10/PC11本项目专供UART4，不能同时使用USART3或SDIO/SD卡占用这两脚。
 * 以下V8.9及更早条目为历史变更，旧PA9/PA10说明以本次UART4接线表为准。
 * V8.9：删除自动按保存路线行驶的网页入口、R1/R2指令、车辆自动控制及云台占用逻辑。
 *   - 保留实时路线绘制、保存路线、打开历史路线、J1/J2/J3记录补传；已有本地JSON文件不改。
 *   - 保留体感云台、体感小车、摇杆小车及云台自动扫描；原速度限制/回中/失联停车不变。
 *   - 基座旧/api/route返回410，不再向STM32转发驾驶指令；路线日志接口继续工作。
 *   - 本次需重烧STM32和基座ESP，刷新2D/3D网页；手持V1.5及摄像头不需重烧。
 *   - 下方全部接线保持不变：TFT、基座、手持3.3V摇杆/ADS/OLED、驱动、编码器及电源。
 * V8.8：仅反转手持摇杆X方向(kXSign=-1)，前后(kYSign=1)与两种体感模式不变。
 *   - 用户确认摇杆VCC改接3.3V后已正常就绪；保留下方所有接线，仅补充供电确认与方向配置。
 *   - 回中/失联/停车/限速保护、H1协议不变；本文件仅更新注释，STM32执行代码不变。
 *   - 只需重烧手持NodeMCU_Handheld_MPU_Car.ino V1.5；无需重烧STM32/基座/摄像头。
 * V8.7：摇杆页显示E00~E15具体原因、X/Y原始ADC、C校准进度、L上次错误和N累计错误。
 *   - 区分ADS地址无响应/读写失败/配置不符/超时/数据越界/中点超范围/抖动/未调度。
 *   - 不增加I2C扫描或采样事务，不放宽校准阈值；三模式、20ms控制协议、回中与失联保护不变。
 *   - 已复核并保留下方全部接线：手持D1/D2共用、ADDR接GND、X/Y接ADS A0/A1、SW接D5。
 *   - 无新引脚，基座/显示屏/电机/编码器/传感器与电源接线不变，本文件仍只改注释。
 *   - 只需重烧手持NodeMCU_Handheld_MPU_Car.ino V1.4；不需重烧STM32/基座/摄像头。
 *   - 原始ADC不是角度或速度；无值显示横线，无效/过期旧值不用于控制；L/N保留到手持复位。
 *   - 共用SDA/SCL持续被拉低时OLED也可能冻结，不能把旧画面当作已刷新故障；需硬件排查。
 * V8.6：ADS1115读取摇杆X/Y，摇杆SW接替原D5独立按键，短按循环三模式，长按0.8秒停车。
 *   - 新接线已补全到下方手持表：ADS与MPU/OLED共用D1/D2；ADDR接GND，X/Y接ADS A0/A1。
 *   - 模块采用3.3V供电/逻辑，旧独立按键拆除；NodeMCU的5V VIN/USB供电规则不变。
 *   - 本地摇杆模式3仍发送H1车模式1；切换epoch递增，STM32原停车/回中、超时、限速保护不改。
 *   - 摇杆模式不叠加MPU输入；摇杆异常/数据超时发送ready=0，重新回中后才允许恢复。
 *   - OLED显示三个本地请求模式及摇杆X/Y百分比；不代表STM32已执行或真实车速。
 *   - 所有基座/TFT/电机/编码器/气体/摄像头实际接线复核不变，本文件只更新注释。
 *   - 只需重烧NodeMCU_Handheld_MPU_Car.ino V1.3；不用重烧32/基座/摄像头或修改网页。
 * V8.5：摄像头最短取图间隔100ms；网页10/5/3帧可选，10帧档105ms留调度余量。
 *   - 摄像头CAMERA_VFLIP=1，仅上下翻转，CAMERA_HMIRROR仍0；网页不再二次翻转。
 *   - 分辨率320x240、单查看者、默认关闭、超时/过期清屏不变；实际帧率/遥控延迟需实测。
 *   - 下方全部接线已复核并保留：摄像头底板USB供电、无需新增STM32信号；其他设备引脚不变。
 *   - 仅需Arduino重烧ESP32CAM_Camera.ino并刷新网页；本main.c仅更新注释，不需重烧32/基座/手持。
 *   - 网页首次默认10帧，已保存的3/5帧偏好保留，可关闭画面后手动切到10帧。
 * V8.4：实物为ESP32-CAM + ESP32-CAM-MB(CH340C)，不是先前资料中的ESP32-S3。
 *   - 新程序ESP32CAM_Camera/ESP32CAM_Camera.ino，采用官方AI Thinker内部摄像头引脚。
 *   - Arduino Board选AI Thinker ESP32-CAM；DIO/40MHz；不使用S3的16MB/OPI配置。
 *   - 完整接线表已复核：仅修正下方摄像头供电、烧录和板内引脚，其他设备接线不变。
 *   - 摄像头HTTP接口仍1.0，现有网页V8.3兼容；只需重烧摄像头，本文件仅注释更新。
 *   - camera-panel.js仅修正摄像头型号提示和接口版本文案，不改两页HTML、收图或遥控逻辑。
 *   - 旧ESP32S3_Camera目录留作历史，不适用本次实物，禁止将旧S3合并固件烧入ESP32。
 *   - 编译与实物验证分开；真实供电稳定性、PSRAM和出图需上电检查。
 * V8.3：仅新增摄像头固件和网页图传；本主函数仅更新版本/接线注释。
 *   - 当时按ESP32-S3资料实现；V8.4已按实物修正。网页直连摄像头HTTP，不通过STM32串口。
 *   - QVGA 320x240、按需最高5fps、默认关闭、单查看者；热点无线带宽仍共用。
 *   - 摄像头独立供电（V8.4已确认ESP32-CAM-MB USB底板）；不接STM32信号，详见下表。
 *   - 只需烧录新摄像头；不需重烧STM32、基座ESP和手持端。
 *   - 固件FIRMWARE_VERSION保留V8.2，避免把注释/网页更新误认为主控控制逻辑更新。
 * V8.2：仅修改路线记录/显示，四轮计数累加80ms后作麦克纳姆位姿估算。
 *   - 每40ms检查轮缘速度跳变；80ms检查前轮和与后轮和的一致性，异常块标记缺口。
 *   - 容差为四个计数加总轮程15%，用于量化/轻微不一致；未作实车标定，不能消除打滑。
 *   - 不强制锁定直线、不删除真实小角度转向；已保存的旧漂移路线不自动修复。
 *   - 自动复走、手控、原始编码器中断/轮向、PWM和无回波允许行驶逻辑全部不变。
 *   - 接线复核：下方手持、基座ESP、TFT、电机、编码器、传感器及电源接线全部不变。
 *   - 编码器仍为1左前PE2/PF8、2右前PE3/PF11、3左后PF6/PD6、4右后PF7/PD7。
 * V8.1：测距完成但无回波时允许启动/继续复走；有效距离不足30cm仍停车。
 *   - 保留探头朝前与600ms稳定等待、测距任务350ms超时、编码器异常及失联停车。
 *   - 无回波代表距离未知，不表示没有障碍；只在有人看护的空地低速试验。
 *   - 编码器累计计数不清零；每次复走自动取当前计数为基准，避免破坏实时轨迹。
 *   - 接线复核：下方手持、基座ESP、TFT、电机、编码器、传感器及电源接线全部不变。
 *   - 排查1/4轮跳变时核对E1A=PE2/E1B=PF8、E4A=PF7/E4B=PD7；不调整方向符号。
 * V8.0：普通路径点连续通过，保留急转弯与终点停车；启动助力在确认转动后退出。
 *   - 编码器故障原因、轮号和采样间隔保留至下次路线上传，实时轮速/PWM仍为当前值。
 *   - 基座串口接收缓存1024字节；忙时保留最新手持帧；网页自动行驶期间降低查询竞争。
 *   - 接线复核：下方手持端、基座ESP、TFT、电机、编码器、气体、灯带、超声波及电源接线全部不变。
 *   - 无新增引脚；手控方向与编码器符号不变；失联、无回波、堵转等停车保护不取消。
 * V7.9：自动路线显式使用自动方向补偿；启动助力提高到22%，仍受30%总PWM上限和反馈保护约束。
 * V7.8：自动路线输出取反后，轮速闭环反馈比较同步取反；手控编码器符号保持原标定。
 * V7.7：实测手控前进 RPM 在 V7.6 下变负，恢复原编码器符号；仅反转自动路线电机指令。
 *   - 手控方向、RPM 标定和已有路线坐标保持原样；自动路线与手控物理前进方向统一。
 * V7.5：自动路线反馈故障保留5秒诊断快照，网页显示具体轮子与原因；不改变手控恢复。
 * V7.4：自动路线故障/停止/完成后释放路线控制权，立即恢复手持控制服务。
 *   - 修复反馈异常后 route_owned 未清零导致体感云台被自动服务分支挡住的问题。
 *   - 自动路线结束或收到停止指令时重置轮速闭环状态，不保留旧故障和输出。
 * V7.3：自动路线轮速闭环增加一次性低速启动助力，便于低目标速度克服电机静摩擦。
 *   - 仅自动路线生效，启动约1.2秒内最高120‰，总PWM仍不超过30%。
 *   - 编码器无脉冲、反向、计数跳变、采样超时仍会立即停车；需要重新烧录STM32。
 * V7.2：网页端 route-repeat.js 更新；固件路线协议和停车保护不变。
 *   - 将记录中的麦克纳姆横移/原地转向段编译为“原地转向 + 前进”控制点。
 *   - 小采样缺口在距离/航向受限时自动插值；大缺口、异常数据仍拒绝复走。
 *   - 不改变 STM32 自动行驶的超声波、失联、编码器、控制超时停车保护。
 * V7.1：STM32独立计算路线，1024条RAM记录约覆盖最近80秒，网页按编号补传。
 *   - 分批校验、只读补传；缓存覆盖/采样异常/主控重启保留缺口，不虚构路线。
 *   - 接线已复核：下方手持、基座、电机、编码器、屏幕及电源接线全部不变。
 *   - 无新增引脚，PD3仍不接；不改变手控方向、PWM或自动行驶停车条件。
 *   - 断电丢失车端RAM；已保存到电脑的路线文件不受影响。
 * V7.0：路线完整上传并校验后由STM32执行，自动模式使用编码器轮速闭环。
 *   - 手控PWM/方向不变；仅自动模式闭环，目标最高18RPM、PWM最高30%。
 *   - 网页续约600ms、手持失联250ms、主循环超时120ms均停车。
 *   - 自动模式探头固定正前方，低于30cm或测距无效停车，不支持自动绕障。
 *   - 前进/转向短路线试验，必须人工对准原起点和朝向；不是SLAM。
 *   - 已核对下方完整接线：无新增引脚，PD3仍不接；编码器/电机/屏幕均不变。
 *   - 编码器：M1 PE2/PF8，M2 PE3/PF11，M3 PF6/PD6，M4 PF7/PD7（A/B）。
 *   - 基座ESP TX->PA10、RX->PA9，稳定3.3V，共地；手持D1/D2、D5不变。
 *   - 使用用户确认存在的电机电源切断开关；软件不代替断电，需架空验证后落地。
 *
 * V6.9.1 变更（2026-09-11，仅网页，下方完整接线及固件代码不变）:
 *   - 恢复“保存路线/打开路线”，兼容旧巡检JSON中的路线；不读取扫描点
 *   - 旧路线与实时接收独立，历史模式可返回实时；不恢复繁琐档案面板
 *   - 保留原有CSV导出和RPM；不需烧录STM32、基座ESP或手持，闭环不变
 *
 * V6.9 变更（2026-09-11，仅网页与说明更新，下方完整接线不变）:
 *   - 按用户要求移除2D/3D新增的巡检档案面板、保存/导入和历史扫描回看
 *   - 保留实时扫描、实时路线、原有轨迹CSV导出、RPM和全部硬件功能
 *   - 不删除桌面“路径保存数据库”已有文件；旧版实现已备份到D盘缓存
 *   - 本次主函数只更新注释，STM32/基座/手持均不需重新烧录，闭环开关不变
 *
 * V6.8 变更（2026-09-11，已复核下方完整手持/基座接线，全部引脚不变）:
 *   - 四轮40ms采样、最多3个样本计算轮速；新增可选前馈+PI控制器
 *   - WheelSpeedConfig.h集中配置；默认WHEEL_SPEED_ENABLE=0，只监测
 *   - 默认继续原手持PWM控制，不增加软件锁；反馈方向需确认后才开启新闭环
 *   - 可选闭环目标上限30RPM、PWM仍不超过30%；增益仅为初始试验值
 *   - 反馈异常时闭环四轮归零，不退回开环；保留失联/回中/换向停顿
 *   - S1诊断每200ms发给基座ESP V4.5；网页显示目标/实测轮速及PWM命令
 *   - 这不是自动复走成品，不新增自动行驶/导入即运行的入口
 *   - STM32和基座需更新才能显示完整诊断；手持V1.2、OLED代码不变
 *
 * V6.7：当时仅网页新增本地巡检点档案，现已在V6.9按用户要求移除。
 *
 * V6.6 变更（2026-09-09，手持OLED及完整接线注释更新）:
 *   - 手持NodeMCU升级V1.2：SSD1306 128x64 I2C OLED并接原MPU总线
 *   - 显示云台/小车本地请求模式、停车请求、WiFi信号、姿态和输入状态
 *   - OLED分块空隙刷新，不改变H1协议/20ms发送/按键/失联处理
 *   - 本次仅需重烧手持端；STM32 V6.5、基座ESP V4.4可继续使用
 *   - STM32控制逻辑、四轮编码器、网页与全部基座引脚均未改变
 *
 * V6.5 变更（2026-09-09，完整接线表已更新）:
 *   - 新增四轮A相上升沿外部中断计数、B相判方向，不占用新定时器
 *   - E1累计计数/时间戳/CRC每80ms发给基座ESP V4.4，网页标定并估算轨迹
 *   - 2D/3D新增行驶轨迹、路程、速度、朝向、四轮计数、标定和导出
 *   - 未提供轮径/计数每圈/车架尺寸时不显示虚构米数；非定位/防滑测量
 *   - 本次重烧STM32及基座ESP；手持NodeMCU V1.1程序无需变化
 *   - V6.4方向修正、30%PWM及V6.3短暂异常恢复均保留
 *
 * V6.4 变更（2026-09-08，本次只改方向符号，引脚不变）:
 *   - 实测前倾变成后退、左倾变成右转，前后和左右方向一起翻转
 *   - 仅调整 CAR_FORWARD_SIGN / CAR_TURN_LEFT_SIGN，保留右侧2/4轮镜像极性
 *   - 只需重烧STM32；手持NodeMCU和基座ESP不变
 *
 * V6.3 变更（2026-09-08，已更新完整接线注释，引脚不变）:
 *   - 短暂无效姿态/过倾立即暂停PWM；300ms内3个新有效包且持续40ms后渐变恢复
 *   - 恢复时任一轴须在50度内；持续异常、250ms失联、模式切换仍需回中
 *   - 短时恢复检查显示CAR:CHECK；保留30%上限、右侧2/4轮反相
 *   - 手持NodeMCU升级V1.1：按键35ms消抖，MPU连续3次读失败才重新初始化
 *   - 本次需烧录STM32和手持NodeMCU；基座ESP V4.3及H1协议不变
 *
 * V6.2 变更（2026-09-07，已复核完整接线表，引脚不变）:
 *   - 小车PWM上限由15%提高到30%，继续使用开环PWM控制
 *   - 保持云台同轴映射：Pitch前后、Roll左右；修正镜像安装右侧2/4轮输出极性
 *   - 回中范围和停止死区统一为8度，回中0.4秒且至少10个新包后自动恢复
 *   - TFT区分LEVEL等待回中、INPUT手持数据无效、TILT倾斜超过55度
 *   - 小车模式显示手持实时ROLL/PITCH，无效数据显示横线，便于判断回中
 *   - 小车模式屏幕角度/链路与电机控制统一按250ms输入时效判断
 *   - 仍无PD3输入；主函数完整保留手持/基座/驱动/屏幕等接线
 *
 * V6.1 变更（2026-09-07，完整接线表已同步更新）:
 *   - 按用户要求解除CAR:LOCK，启用四轮架空试转；最大PWM由35%降至15%
 *   - 移除未接线的PD3急停输入及其检测；PD3不接，不需要短接到GND
 *   - 不把未验证的轮向/硬件急停伪记为已确认；本版不代表实物验收完成
 *   - 保留250ms失联停车、回中确认、按键软件停车、缓启动和换向停顿
 *   - 保留TIM7主循环超时及异常PWM归零；软件不能代替直接切断电机电源
 *   - 只需重烧STM32；手持NodeMCU和基座ESP固件/协议均不改
 *
 * V6.0 变更（2026-09-06，完整接线表已同步更新）:
 *   - NodeMCU V3 + MPU6050 + D5按键，短按切换云台/小车，长按软件停车
 *   - 新增四路麦克纳姆轮PWM+DIR驱动，1左前/2右前/3左后/4右后
 *   - TIM8 PC6/PC7 + TIM9 PE5/PE6，避开板载SRAM及原有舵机/灯带资源
 *   - H1带CRC/序号/有效标志；250ms失联停车、回中确认、限速/缓启动/换向停顿
 *   - TIM7独立监测主循环超时；异常处理直接关闭电机PWM
 *   - 当时电机默认未解锁；V6.1已改为15% PWM上限的架空试转
 *   - 气体G2、超声波、屏幕、舵机、语音及灯带功能保留
 *
 * V5.3 变更（已复核下方完整接线表，本次不改引脚）:
 *   - 修正气体18位浓度、小数位、单位、状态和气体类型解码
 *   - 区分模块在线、浓度有效、预热/故障/报警；失联不保留正常状态
 *   - USART1/USART2改为有界中断发送，气体查询带超时与帧校验
 *   - 气体遥测升级G2帧，需同时更新基座ESP V4.2；手持端不变
 *   - 温湿度编码未证实，网页不再将原始寄存器误显示成温湿度
 *
 * V5.2 变更:
 *   - USART2(PA2/PA3)接入4MZ-HH4四合一气体检测模块
 *   - 采用9600/8N1 Modbus-RTU，中断接收和非阻塞轮询，避免阻塞云台主循环
 *   - 气体浓度、单位、小数位、状态、气体类型、温湿度通过独立G帧发送到基座ESP
 *
 * V5.1 变更:
 *   - DY-SV17F第三路危险语音触发引脚由未引出的PG2迁移到PC3
 *   - PC3为开发板排针已引出的独立GPIO，语音触发逻辑保持不变
 *
 * V5.0 变更:
 *   - 增加 DY-SV17F 三条语音播报，模块使用 I/O 独立模式0
 *   - PB10低脉冲播放00001.mp3（距离进入20~40 cm预警区）
 *   - PB11低脉冲播放00002.mp3（网页进入全方位扫描模式）
 *   - PC3低脉冲播放00003.mp3（距离进入小于20 cm危险区）
 *   - 全方位扫描语音仅在模式切换时触发一次，扫描过程中不会重复播报
 *   - 距离语音复用20/22 cm和40/42 cm迟滞阈值，不会持续重复触发
 *   - 三路触发均采用非阻塞状态机，危险语音优先级最高
 *
 * V4.0 变更:
 *   - 手持、左右、上下和全方位扫描统一按测距结果显示灯带颜色
 *   - 仅保留小于20 cm红色闪烁、20~40 cm黄色、40 cm以上绿色
 *   - 移除启动、失联、扫描、无回波对应的蓝/青/白/紫状态色
 *   - 短暂等待测距时保留上次距离颜色，持续无有效回波后熄灭
 *
 * V3.0 变更:
 *   - 增加 5V WS2812 RGB灯带控制，30颗灯珠整体同色
 *   - 使用 PB6 / TIM4_CH1 + DMA1_Stream0 输出 800 kHz 数据
 *   - 一条30灯灯带剪为两段，每段15颗；数据串联，仍只占用一个PB6接口
 *   - 灯带亮度限制为约 25%，降低30颗灯珠的供电压力
 *   - 增加红色距离报警、黄色预警、扫描色、无回波和手持失联状态灯
 *   - 距离状态增加迟滞和短时保持，减少20/40 cm附近颜色抖动
 *
 * V2.0 变更:
 *   - 增加俯仰舵机机械零位下压 2度补偿
 *   - 逻辑俯仰角仍以探头水平为 0度，网页和屏幕显示不受机械补偿影响
 *   - 全方位扫描俯仰层距由 10度减小为 5度，扫描由 7层加密为 13层
 *
 * V1.0 变更:
 *   - ESP-01S 网页/手持双模式控制
 *   - 左右、上下和全方位栅格扫描
 *   - HC-SR04 测距、报警和网页遥测
 *   - 自动扫描采用“舵机停止后测距”，减少运动拖影
 *   - 手腕 Pitch -2度零点校准
 *
 * 注意: 网页点云的 6.5 cm 机械偏心补偿位于 HTML 文件中，
 *       仅烧录 STM32 不会更新已打开的旧网页。
 */

/**
 * 二维舵机云台 — MPU6050体感控制 + 超声波避障提示
 * 板子: 正点原子 STM32F407ZG (8MHz晶振)
 *
 * 完整接线（V8.13/2026-10-02：基座USART2 PA2/PA3；气体UART4 PC10/PC11）:
 *
 *   ESP32-CAM车载摄像头（实物ESP-32S，不是ESP32-S3；ESP32-CAM-MB/CH340C底板）:
 *     程序 -> ESP32CAM_Camera/ESP32CAM_Camera.ino V1.2；接线见V8.4说明，帧率升级见V8.5说明
 *     摄像头排线 -> 本板摄像头插座，断电插拔并锁紧，不接STM32 DCMI接口
 *     摄像头板 -> ESP32-CAM-MB底板，核对排针标识、不反插/不错位
 *     底板Micro-USB -> 电脑USB数据口（下载）或稳定5V USB电源（运行），无需另接TTL
 *     脱离底板装车 -> 稳压5V接明确标注5V的针脚；电源GND接GND；建议电源额定至少1A留余量
 *     不使用旁边有3V3/5V选择焊盘的VCC脚；不把5V接到3V3；不要USB与外部5V同时供电
 *     备用USB-TTL TX(3.3V逻辑) -> U0R/GPIO3；TTL RX <- U0T/GPIO1；TTL GND -> GND
 *     下载 -> 按住底板IO0，按一下RST，进入下载后松开IO0，再点击Upload
 *     下载后 -> IO0保持松开/GPIO0不接GND，再按RST；RST不可长期接地
 *     GPIO0正常运行时是摄像头XCLK，不能保持下载短接；GPIO2不另接高电平
 *     与STM32/基座ESP/手持端 -> 不接TX/RX或其他信号线，仅通过WiFi通信
 *     无串口连接、独立电源时不需要另拉通信地线；用TTL烧录时须与TTL共地
 *     禁止摄像头接电机Vin/电池原电压、禁止从ESP-01S 3.3V引脚取电
 *     WiFi -> GimbalCtrl / <LOCAL_WIFI_PASSWORD>；IP由基座DHCP分配，看串口Camera URL
 *     不固定为192.168.4.1（该地址属于基座），一个摄像头占一个热点名额
 *     摄像头内部AI Thinker映射：PWDN32、RESET=-1、XCLK0、SDA26、SCL27、VSYNC25、HREF23、PCLK22
 *     D0~D7 -> GPIO5/18/19/21/36/39/34/35（板内排线，不需外接）
 *     GPIO4闪光灯保持关闭；不使用TF卡；不得套用S3_EYE引脚/OPI PSRAM/16MB配置
 *     画面中断/过期须人工停止驾驶；视频不产生自动停车或运动指令
 *
 *   ST7735S 1.77寸14PIN TFT（软件SPI）:
 *     1 GND      -> GND
 *     2 VCC      -> 3.3V
 *     3 SCK      -> PG6
 *     4 SDA      -> PG7
 *     5 RES      -> PG13
 *     6 RS/DC    -> PG11
 *     7 CS       -> GND（单屏专用，固定选中）
 *     8/9 LEDA   -> 3.3V（串联100~220欧限流电阻，不接MCU GPIO）
 *     10~13 GT_* -> 不接（触摸接口）
 *     14 GND     -> GND
 *
 *   底座ESP-01S（USART2，115200/8N1，STM32与ESP双向通信）:
 *     程序          -> ESP01S_Base_UDP.ino V4.9（本次不改ESP程序，不需重烧ESP）
 *     VCC、EN/CH_PD -> 稳定3.3V
 *     GND           -> STM32 GND
 *     TX            -> PA3 (USART2_RX，ESP把手持姿态送给STM32)
 *     RX            -> PA2 (USART2_TX，发送姿态、测距、G2/E1/S1及车辆状态)
 *     GPIO0         -> 运行时不要接GND
 *     原PA9/PA10    -> 撤掉ESP信号线；本版不再初始化USART1，不会在那里输出遥测
 *     PA10-TXD/PA9-RXD跳线帽不再参与ESP通信；可以保持原样，不需要为USART2改动
 *     PA2/PA3       -> 仅接基座ESP；PC10/PC11仅接下方气体模块
 *     先断电核对PCB接线，再重烧STM32 V8.13；VCC/EN/GND接线不变
 *
 *   手持端已换 NodeMCU V3 CH340G（不再用旧ESP-01S手持程序）:
 *     程序 -> NodeMCU_Handheld_MPU_Car.ino V1.6（V8.13不改手持，不需重烧）
 *     烧录/供电 -> 板载USB接电脑/稳定5V USB电源；不要往3V3脚接5V
 *     不用USB时 -> 稳定5V正极接VIN、负极接GND；USB与外部5V不同时供电
 *     MPU6050 VCC -> NodeMCU 3V3；GND -> NodeMCU GND
 *     MPU6050 SDA -> D2(GPIO4)；SCL -> D1(GPIO5)；AD0 -> GND
 *     MPU6050 INT/XDA/XCL -> 不接
 *     SSD1306 128x64四针I2C OLED VCC -> 3V3；GND -> NodeMCU GND
 *     OLED SDA -> D2(GPIO4)；SCL -> D1(GPIO5)，与MPU对应引脚并接
 *     OLED须支持3.3V供电；I2C上拉只能到3.3V，不接5V；默认0x3C/0x3D探测
 *     此驱动不适用SPI/SH1106/其他分辨率；需先核对屏幕型号
 *     OLED显示手持请求，不是STM32已执行反馈；缺屏可遥控，共用总线短路会影响MPU
 *     V1.5摇杆页 -> E当前诊断/X-Y原始ADC/C校准进度/L上次错误/N累计错误，S/W为上次错误细节
 *     EN/RST -> 保持NodeMCU板载电路，无需外接；GPIO0/GPIO2正常运行不要接地
 *     ADS1115 VDD/VCC -> NodeMCU 3V3；GND -> NodeMCU GND
 *     ADS1115 SDA -> D2(GPIO4)；SCL -> D1(GPIO5)，与MPU/OLED对应线并联
 *     ADS1115 ADDR -> GND（固定0x48）；A2/A3/ALERT/RDY -> 不接
 *     普通双电位器摇杆 VCC -> NodeMCU 3V3；GND -> NodeMCU GND；用户已确认3.3V可正常就绪
 *     摇杆 X/VRx -> ADS1115 A0/AIN0；Y/VRy -> ADS1115 A1/AIN1（不是NodeMCU A0）
 *     摇杆 SW -> D5(GPIO14)，按下接地、内部上拉；SW/X/Y不得送入5V
 *     原独立按键 -> 拆除，由摇杆SW代替，不再需要第二个按钮
 *     短按松开循环 体感云台 -> 体感小车 -> 摇杆小车；长按0.8秒软件停车
 *     停车后松开不会自动恢复；再短按返回体感云台；断网也返回体感云台
 *     上电手持保持水平静止、摇杆松开约2秒；切入摇杆小车/摇杆故障恢复须先回中
 *     摇杆方向 -> HandheldInputConfig.h：kXSign=-1修正左右反向，kYSign=1保留前后方向
 *     仅摇杆小车模式X轴反转，体感云台/体感小车不变；烧录后架空核对左右方向
 *     I2C上拉只能到3.3V；普通ADS芯片量程设4.096V不代表允许3.3V供电下输入5V
 *     V1.1按键连续稳定35ms后才判定按下/松开，短毛刺不会清掉姿态有效标志
 *     这是软件控制按键，不能切断电机电源；D标签不是GPIO编号
 *
 *   MD-TB6612稳压版四路电机板（仅适用照片中的PWM+DIR接口）:
 *     M1PWM -> PC6(TIM8_CH1)；M1DIR -> PC4；M1电机 -> 1左前轮
 *     M2PWM -> PC7(TIM8_CH2)；M2DIR -> PC5；M2电机 -> 2右前轮
 *     M3PWM -> PE5(TIM9_CH1)；M3DIR -> PB12；M3电机 -> 3左后轮
 *     M4PWM -> PE6(TIM9_CH2)；M4DIR -> PB13；M4电机 -> 4右后轮
 *     V6.2方向极性：1/3轮+1，2/4轮-1；通过软件修正，以上接线不变
 *     驱动板GND -> STM32 GND，与电池负极共地
 *     每路PWM在驱动板输入端加10k欧下拉到GND，防上电/复位悬空误转
 *     驱动板Vin+/- -> 带保险且能随时直接断电的电池正/负，供电须同时
 *       满足板子6~13V范围和电机额定电压，并核对启动/堵转电流
 *     PD3 -> 不接；V6.1按用户要求移除硬件急停输入，不需要短接GND
 *     本版已解除软件锁，V6.4保留最大PWM为30%；四轮架空、固定车架试方向
 *     回中判定：ROLL在-8~8度，PITCH在-10~6度（含-2度零点补偿）
 *     V6.4短暂异常300ms内可确认后渐变恢复；失联/切模式/持续异常仍需回中
 *       无硬件急停反馈，手持软件停车不能替代电机电源断开；禁止GPIO接电池电压
 *     编码器E1A/B~E4A/B -> 接下方编码器表；ADC仍不接；V6.8默认监测+原开环PWM
 *     新闭环开关/CPR/方向见WheelSpeedConfig.h，不能用网页标定自动改变主控参数
 *     电机两根动力线只接各路M+/M-；不要按线色猜编码器V/GND/A/B
 *     驱动板5V/3V3排针是电源输出，不接STM32信号脚，不并联两路电源输出
 *     不使用旧F103示例的AIN1/AIN2/STBY接法，也不占用板上SRAM引脚
 *
 *   V6.5 四轮AB霍尔编码器（通过驱动板编码器排针引出，共8根信号线）:
 *     E1A -> PE2(EXTI2)；E1B -> PF8，1左前轮
 *     E2A -> PE3(EXTI3)；E2B -> PF11，2右前轮
 *     E3A -> PF6(EXTI6)；E3B -> PD6，3左后轮
 *     E4A -> PF7(EXTI7)；E4B -> PD7，4右后轮
 *     六芯电机插头仍接驱动板；驱动板已供编码器电源时不再并接另一组电源
 *     编码器VCC/GND由驱动板提供并与STM32共地；A/B按3.3V电平设计
 *     若驱动板A/B输出为5V，先做电平转换，禁止将5V直接接这些编码器输入
 *     PF11/PD6/PD7不同时连接旧触摸/摄像头模块；本项目未使用这些接口
 *     编码器A上升沿计一次（1倍频）；11PPR资料通常对应每轮圈11乘减速比
 *     用户提供：轮径97mm、轮距202mm、轴距210mm、减速比34:1 -> 374次/轮圈
 *     网页已预填以上参数；四轮方向仍需实际核对，每圈计数建议转轮复核
 *     本功能记录轮子转动估算轨迹，无法测出打滑；无脉冲不能判断是否断线
 *
 *   S20F舵机:
 *     Roll信号      -> PA1  (TIM2_CH2)
 *     Pitch信号     -> PA7  (TIM3_CH2)
 *     舵机电源      -> 独立电源，电压按S20F规格
 *     舵机GND       -> 独立电源GND和STM32 GND必须共地
 *
 *   HC-SR04:
 *     VCC -> 5V    GND -> GND    Trig -> PC0    Echo -> PC1
 *
 *   4MZ-HH4四合一气体检测模块（UART4，9600/8N1，Modbus-RTU）:
 *     模块+ -> 稳定5V（4.5~5.5V，不能接STM32 3.3V）
 *     模块- -> STM32 GND（与整机共地）
 *     模块R/RX -> PC10（UART4_TX，STM32发送查询）
 *     模块T/TX -> PC11（UART4_RX，模块返回数据）
 *     串口信号是3.3V TTL，供电5V不代表串口可以接5V电平。
 *     默认地址：01=CH4，02=O2，03=CO，04=H2S
 *     模块上电后建议等待约180秒稳定；页面预热状态来自模块状态码，
 *     并非已完成180秒的证明。气体专用UART4；不得接到USART2/ESP串口。
 *     PC10/PC11不得同时连接USART3或SDIO/SD卡；PA2/PA3现为基座ESP专用。
 *
 *   有源蜂鸣器DZQJ（低电平响）:
 *     VCC -> 3.3V  GND -> GND    I/O -> PC2
 *
 *   DY-SV17F语音模块（I/O独立模式0，三条语音）:
 *     V5      -> 独立稳定5V电源（不能接STM32的3.3V）
 *     GND     -> 5V电源GND和STM32 GND，必须共地
 *     SPK+/-  -> 4~8欧、推荐3~5W喇叭的两端；SPK-不能接GND
 *     CON3    -> 10k欧 -> GND（上电选择模式，之后该脚为BUSY，禁止直接接地）
 *     CON2    -> 10k欧 -> 模块V33
 *     CON1    -> 10k欧 -> GND
 *     IO0     -> PB10（00001预警）；IO0再通过10k欧上拉到模块V33
 *     IO1     -> PB11（00002全方位扫描）；IO1再通过10k欧上拉到模块V33
 *     IO2     -> PC3 （00003危险）；IO2再通过10k欧上拉到模块V33
 *     PB10/PB11/PC3只负责拉低，禁止把IO0/IO1/IO2上拉到5V
 *     V33     -> 只给模式和IO上拉使用，不给外部负载供电
 *     音频文件必须放在模块Flash根目录，并严格命名为
 *               00001.mp3、00002.mp3、00003.mp3
 *
 *   WS2812 RGB灯带（两段各15颗、5V供电、整体同色）:
 *     PB6 -> 第一段 DIN；第一段 DOUT -> 第二段 DIN；第二段 DOUT不接
 *     两段+5V分别并联到稳定5V电源（不要接STM32的3.3V引脚）
 *     两段GND、5V电源GND和STM32 GND必须共地
 *     直接连接时数据线尽量短于10 cm，断电接线，禁止把5V接到PB6
 *
 *   ST-Link（烧录/调试）:
 *     SWDIO -> PA13    SWCLK -> PA14    GND -> GND
 *     3.3V  -> 板端3.3V电平参考；开发板已供电时不要重复供电
 *
 *   MPU6050现在接手持端NodeMCU；底座PB10/PB11和PC3用于DY-SV17F触发。
 */
#include "stm32f4xx.h"
#include "Delay.h"
#include "TFT_ST7735.h"
#include "WirelessControl.h"
#include "GasSensor.h"
#include "CarMotor.h"
#include "CarConfig.h"
#include "WheelEncoder.h"
#include "RouteLog.h"
#include "WS2812.h"
#include <math.h>

#define FIRMWARE_VERSION "V8.13"

/* ========== HC-SR04 ========== */
#define TRIG_H()  GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define TRIG_L()  GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define ECHO_RD() ((GPIOC->IDR & GPIO_Pin_1) ? 1 : 0)
#define SONIC_PERIOD_MS      80U
#define SCAN_STEP_PERIOD_MS 150U
#define SCAN_SETTLE_MS       70U
#define HANDHELD_ANGLE_GAIN       3.0f
#define HANDHELD_PITCH_ZERO_DEG (-2.0f)
#define PITCH_SERVO_ZERO_TRIM_DEG  2.0f
#define FULL_SCAN_PITCH_STEP_DEG   5.0f

/* ========== WS2812状态灯 ========== */
#define LED_DANGER_BLINK_MS         250U
#define LED_DISTANCE_HOLD_MS        500U
#define LED_DANGER_ENTER_CM        20.0f
#define LED_DANGER_EXIT_CM         22.0f
#define LED_WARNING_ENTER_CM       40.0f
#define LED_WARNING_EXIT_CM        42.0f

/* ========== 蜂鸣器 DZQJ (PC2, 低电平响) ========== */
#define BUZZ_ON()  GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define BUZZ_OFF() GPIO_SetBits(GPIOC, GPIO_Pin_2)

/* ========== DY-SV17F语音模块（PB10/PB11/PC3，低脉冲触发） ========== */
#define VOICE_ALL_RELEASE()                              \
    do                                                   \
    {                                                    \
        GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11); \
        GPIO_SetBits(GPIOC, GPIO_Pin_3);                 \
    } while (0)
#define VOICE_TRACK1_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define VOICE_TRACK2_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define VOICE_TRACK3_LOW() GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define VOICE_FILE_WARNING   1U
#define VOICE_FILE_SCAN_ALL  2U
#define VOICE_FILE_DANGER    3U
#define VOICE_STARTUP_MS 1000U
#define VOICE_RELEASE_MS   10U
#define VOICE_PULSE_MS    100U

/* ========== DWT微秒延时 ========== */
static void DWT_Delay_us(uint32_t us)
{
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t wait_cycles = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start_cycles) < wait_cycles);
}

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ========== 超声波测距 (cm) ========== */
static void HC_SR04_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PC0=Trig 推挽输出 */
    g.GPIO_Pin = GPIO_Pin_0;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &g);
    TRIG_L();

    /* PC1=Echo 浮空输入 */
    g.GPIO_Pin = GPIO_Pin_1;
    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &g);
}

static float HC_SR04_GetDistance(void)
{
    uint32_t wait_start_cycles;
    uint32_t echo_start_cycles;
    uint32_t echo_cycles;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    /* 发送10us触发脉冲 */
    TRIG_H();
    DWT_Delay_us(10);
    TRIG_L();

    /* 等待Echo变高 (超时20ms, 对应~3.4m) */
    wait_start_cycles = DWT->CYCCNT;
    while (!ECHO_RD())
    {
        if ((uint32_t)(DWT->CYCCNT - wait_start_cycles) >=
            20000U * cycles_per_us)
            return -1.0f;
    }

    echo_start_cycles = DWT->CYCCNT;

    /* 等待Echo变低 (超时25ms, 对应~4.2m) */
    while (ECHO_RD())
    {
        if ((uint32_t)(DWT->CYCCNT - echo_start_cycles) >=
            25000U * cycles_per_us)
            return -1.0f;
    }

    echo_cycles = (uint32_t)(DWT->CYCCNT - echo_start_cycles);

    /* 距离 = 脉宽(us) / 58 (cm) */
    return (float)echo_cycles / ((float)cycles_per_us * 58.0f);
}

/* ========== 蜂鸣器 ========== */
static void Buzzer_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PC2=蜂鸣器 推挽输出 */
    g.GPIO_Pin = GPIO_Pin_2;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &g);
    BUZZ_OFF();
}

typedef enum
{
    VOICE_TRIGGER_IDLE = 0,
    VOICE_TRIGGER_RELEASE,
    VOICE_TRIGGER_LOW
} VoiceTriggerState_t;

typedef enum
{
    VOICE_DISTANCE_UNKNOWN = 0,
    VOICE_DISTANCE_SAFE,
    VOICE_DISTANCE_WARNING,
    VOICE_DISTANCE_DANGER
} VoiceDistanceZone_t;

static VoiceTriggerState_t voice_trigger_state = VOICE_TRIGGER_IDLE;
static uint8_t voice_active_track;
static uint8_t voice_pending_track;
static uint32_t voice_state_cycles;
static uint32_t voice_init_cycles;
static uint8_t voice_startup_complete;

static void Voice_Init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC, ENABLE);

    /* 先把ODR置高再切成输出，避免初始化时产生误触发低脉冲。 */
    VOICE_ALL_RELEASE();
    g.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_OD;
    g.GPIO_Speed = GPIO_Speed_25MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &g);

    g.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOC, &g);
    VOICE_ALL_RELEASE();

    voice_trigger_state = VOICE_TRIGGER_IDLE;
    voice_active_track = 0U;
    voice_pending_track = 0U;
    voice_state_cycles = DWT->CYCCNT;
    voice_init_cycles = voice_state_cycles;
    voice_startup_complete = 0U;
}

static void Voice_Request(uint8_t track)
{
    if (track == VOICE_FILE_DANGER)
    {
        /* 危险报警优先；可覆盖尚未开始的预警或扫描请求。 */
        if (voice_active_track != VOICE_FILE_DANGER)
            voice_pending_track = VOICE_FILE_DANGER;
    }
    else if (track == VOICE_FILE_WARNING &&
             voice_active_track == 0U && voice_pending_track == 0U)
    {
        voice_pending_track = VOICE_FILE_WARNING;
    }
    else if (track == VOICE_FILE_SCAN_ALL &&
             voice_active_track != VOICE_FILE_DANGER &&
             voice_pending_track != VOICE_FILE_DANGER)
    {
        /* 全方位扫描播报优先级低于危险报警。 */
        voice_pending_track = VOICE_FILE_SCAN_ALL;
    }
}

static void Voice_Service(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t elapsed_ms =
        (uint32_t)(now - voice_state_cycles) /
        (SystemCoreClock / 1000U);

    if (!voice_startup_complete)
    {
        if ((uint32_t)(now - voice_init_cycles) <
            (SystemCoreClock / 1000U) * VOICE_STARTUP_MS)
            return;
        voice_startup_complete = 1U;
    }

    if (voice_trigger_state == VOICE_TRIGGER_IDLE)
    {
        if (voice_pending_track == 0U)
            return;

        voice_active_track = voice_pending_track;
        voice_pending_track = 0U;
        VOICE_ALL_RELEASE();
        voice_state_cycles = now;
        voice_trigger_state = VOICE_TRIGGER_RELEASE;
    }
    else if (voice_trigger_state == VOICE_TRIGGER_RELEASE)
    {
        if (elapsed_ms < VOICE_RELEASE_MS)
            return;

        if (voice_active_track == VOICE_FILE_WARNING)
            VOICE_TRACK1_LOW();
        else if (voice_active_track == VOICE_FILE_SCAN_ALL)
            VOICE_TRACK2_LOW();
        else if (voice_active_track == VOICE_FILE_DANGER)
            VOICE_TRACK3_LOW();
        else
        {
            VOICE_ALL_RELEASE();
            voice_trigger_state = VOICE_TRIGGER_IDLE;
            return;
        }

        voice_state_cycles = now;
        voice_trigger_state = VOICE_TRIGGER_LOW;
    }
    else
    {
        if (elapsed_ms < VOICE_PULSE_MS)
            return;

        VOICE_ALL_RELEASE();
        voice_active_track = 0U;
        voice_state_cycles = now;
        voice_trigger_state = VOICE_TRIGGER_IDLE;
    }
}

static void VoiceAlarm_Update(uint8_t distance_valid, float distance)
{
    static VoiceDistanceZone_t zone = VOICE_DISTANCE_UNKNOWN;
    VoiceDistanceZone_t previous_zone;

    /* 无回波期间保持原状态，避免一次丢测造成重复播报。 */
    if (!distance_valid)
        return;

    previous_zone = zone;
    if (zone == VOICE_DISTANCE_DANGER)
    {
        if (distance >= LED_DANGER_EXIT_CM)
            zone = (distance < LED_WARNING_EXIT_CM) ?
                   VOICE_DISTANCE_WARNING : VOICE_DISTANCE_SAFE;
    }
    else if (zone == VOICE_DISTANCE_WARNING)
    {
        if (distance < LED_DANGER_ENTER_CM)
            zone = VOICE_DISTANCE_DANGER;
        else if (distance >= LED_WARNING_EXIT_CM)
            zone = VOICE_DISTANCE_SAFE;
    }
    else
    {
        if (distance < LED_DANGER_ENTER_CM)
            zone = VOICE_DISTANCE_DANGER;
        else if (distance < LED_WARNING_ENTER_CM)
            zone = VOICE_DISTANCE_WARNING;
        else
            zone = VOICE_DISTANCE_SAFE;
    }

    /* 模块上电准备期间只保留当前距离对应的最新请求。 */
    if (!voice_startup_complete)
    {
        if (zone == VOICE_DISTANCE_DANGER)
            voice_pending_track = VOICE_FILE_DANGER;
        else if (zone == VOICE_DISTANCE_WARNING)
            voice_pending_track = VOICE_FILE_WARNING;
        else
            voice_pending_track = 0U;
        return;
    }

    if (zone == VOICE_DISTANCE_DANGER &&
        previous_zone != VOICE_DISTANCE_DANGER)
    {
        Voice_Request(VOICE_FILE_DANGER);
    }
    else if (zone == VOICE_DISTANCE_WARNING &&
             (previous_zone == VOICE_DISTANCE_SAFE ||
              previous_zone == VOICE_DISTANCE_UNKNOWN))
    {
        Voice_Request(VOICE_FILE_WARNING);
    }
}

/* ========== 舵机PWM ========== */
static void Servo_PWM_Init(void)
{
    GPIO_InitTypeDef g;
    TIM_TimeBaseInitTypeDef t;
    TIM_OCInitTypeDef oc;

    /* TIM2_CH2 (PA1) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    g.GPIO_Pin = GPIO_Pin_1; g.GPIO_Mode = GPIO_Mode_AF; g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &g);
    t.TIM_Prescaler = 84 - 1; t.TIM_Period = 20000 - 1;
    t.TIM_ClockDivision = TIM_CKD_DIV1; t.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &t);
    oc.TIM_OCMode = TIM_OCMode_PWM1; oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 1500; oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &oc);
    TIM_Cmd(TIM2, ENABLE);

    /* TIM3_CH2 (PA7) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
    g.GPIO_Pin = GPIO_Pin_7; GPIO_Init(GPIOA, &g);
    TIM_TimeBaseInit(TIM3, &t);
    TIM_OC2Init(TIM3, &oc);
    TIM_Cmd(TIM3, ENABLE);
}

static uint16_t AngleToPulse(float angle)
{
    if (angle < -90.0f) angle = -90.0f;
    if (angle >  90.0f) angle =  90.0f;
    return (uint16_t)(1500 + (int32_t)(angle * 1000 / 90));
}

static uint16_t PitchToPulse(float pitch)
{
    /* 逻辑 Pitch 保持 0度；只在PWM输出端补偿舵机/舵盘机械零位。 */
    return AngleToPulse(-pitch + PITCH_SERVO_ZERO_TRIM_DEG);
}

static uint8_t Scan_Update(WirelessControlMode_t mode,
                           float *roll, float *pitch,
                           int8_t *direction,
                           uint32_t *last_step_cycles)
{
    const uint32_t step_cycles =
        (SystemCoreClock / 1000U) * SCAN_STEP_PERIOD_MS;

    if ((uint32_t)(DWT->CYCCNT - *last_step_cycles) < step_cycles)
        return 0;
    *last_step_cycles = DWT->CYCCNT;

    if (mode == WIRELESS_MODE_SCAN_HORIZONTAL)
    {
        *pitch = 0.0f;
        *roll += 2.0f * (float)(*direction);
        if (*roll >= 60.0f) { *roll = 60.0f; *direction = -1; }
        if (*roll <= -60.0f) { *roll = -60.0f; *direction = 1; }
    }
    else if (mode == WIRELESS_MODE_SCAN_VERTICAL)
    {
        *roll = 0.0f;
        *pitch += 2.0f * (float)(*direction);
        if (*pitch >= 30.0f) { *pitch = 30.0f; *direction = -1; }
        if (*pitch <= -30.0f) { *pitch = -30.0f; *direction = 1; }
    }
    else if (mode == WIRELESS_MODE_SCAN_ALL)
    {
        *roll += 2.0f * (float)(*direction);
        if (*roll >= 60.0f)
        {
            *roll = 60.0f;
            *direction = -1;
            *pitch += FULL_SCAN_PITCH_STEP_DEG;
            if (*pitch > 30.0f) *pitch = -30.0f;
        }
        else if (*roll <= -60.0f)
        {
            *roll = -60.0f;
            *direction = 1;
            *pitch += FULL_SCAN_PITCH_STEP_DEG;
            if (*pitch > 30.0f) *pitch = -30.0f;
        }
    }
    else
    {
        return 0;
    }

    return 1;
}

typedef enum
{
    LED_DISTANCE_UNKNOWN = 0,
    LED_DISTANCE_SAFE,
    LED_DISTANCE_WARNING,
    LED_DISTANCE_DANGER
} LedDistanceZone_t;

static void StatusLed_GetColor(LedDistanceZone_t zone, uint8_t visible,
                               uint8_t *red, uint8_t *green, uint8_t *blue)
{
    if (!visible || zone == LED_DISTANCE_UNKNOWN)
    {
        *red = 0U; *green = 0U; *blue = 0U;
        return;
    }

    switch (zone)
    {
        case LED_DISTANCE_DANGER:
            *red = 255U; *green = 0U;   *blue = 0U;
            break;
        case LED_DISTANCE_WARNING:
            *red = 255U; *green = 140U; *blue = 0U;
            break;
        case LED_DISTANCE_SAFE:
            *red = 0U;   *green = 255U; *blue = 0U;
            break;
        case LED_DISTANCE_UNKNOWN:
        default:
            *red = 0U;   *green = 0U;   *blue = 0U;
            break;
    }
}

static void StatusLed_Service(uint8_t distance_valid,
                              float distance)
{
    static uint8_t initialized;
    static uint8_t blink_visible = 1U;
    static uint8_t color_dirty = 1U;
    static uint32_t last_blink_cycles;
    static uint32_t last_distance_cycles;
    static LedDistanceZone_t distance_zone = LED_DISTANCE_UNKNOWN;
    static LedDistanceZone_t previous_zone = LED_DISTANCE_UNKNOWN;
    uint32_t now = DWT->CYCCNT;
    uint8_t distance_recent;
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    WS2812_Service();

    if (!initialized)
    {
        initialized = 1U;
        last_blink_cycles = now;
    }

    if (distance_valid)
    {
        if (distance_zone == LED_DISTANCE_DANGER)
        {
            if (distance >= LED_DANGER_EXIT_CM)
                distance_zone = (distance < LED_WARNING_EXIT_CM) ?
                                LED_DISTANCE_WARNING : LED_DISTANCE_SAFE;
        }
        else if (distance_zone == LED_DISTANCE_WARNING)
        {
            if (distance < LED_DANGER_ENTER_CM)
                distance_zone = LED_DISTANCE_DANGER;
            else if (distance >= LED_WARNING_EXIT_CM)
                distance_zone = LED_DISTANCE_SAFE;
        }
        else
        {
            if (distance < LED_DANGER_ENTER_CM)
                distance_zone = LED_DISTANCE_DANGER;
            else if (distance < LED_WARNING_ENTER_CM)
                distance_zone = LED_DISTANCE_WARNING;
            else
                distance_zone = LED_DISTANCE_SAFE;
        }
        last_distance_cycles = now;
    }

    distance_recent = (uint8_t)(distance_valid ||
        (distance_zone != LED_DISTANCE_UNKNOWN &&
         (uint32_t)(now - last_distance_cycles) <=
             (SystemCoreClock / 1000U) * LED_DISTANCE_HOLD_MS));

    if (!distance_recent)
        distance_zone = LED_DISTANCE_UNKNOWN;

    if (distance_zone != previous_zone)
    {
        previous_zone = distance_zone;
        blink_visible = 1U;
        last_blink_cycles = now;
        color_dirty = 1U;
    }

    if (distance_zone == LED_DISTANCE_DANGER &&
        (uint32_t)(now - last_blink_cycles) >=
            (SystemCoreClock / 1000U) * LED_DANGER_BLINK_MS)
    {
        last_blink_cycles = now;
        blink_visible = (uint8_t)!blink_visible;
        color_dirty = 1U;
    }

    if (color_dirty && !WS2812_IsBusy())
    {
        StatusLed_GetColor(distance_zone, blink_visible,
                           &red, &green, &blue);
        if (WS2812_ShowColor(red, green, blue))
            color_dirty = 0U;
    }
}

/* ========== 全局 ========== */
float S1_Angle, S2_Angle, S1_Filtered, S2_Filtered;

/* ========== 主函数 ========== */
int main(void)
{
    uint32_t last_sonic_cycles = 0;
    uint32_t last_display_cycles = 0;
    uint32_t last_telemetry_cycles = 0;
    uint32_t last_gas_telemetry_cycles = 0;
    uint32_t last_wheel_speed_cycles = 0;
    uint32_t last_scan_step_cycles = 0;
    uint32_t telemetry_sequence = 0;
    uint32_t gas_telemetry_sequence = 0;
    uint32_t wheel_speed_sequence = 0;
    uint8_t gas_telemetry_channel = 0;
    float distance = 0;
    uint8_t buzzer_on = 0;
    uint8_t link_was_ok = 0;
    uint8_t have_control_packet = 0;
    uint8_t have_wireless = 0;
    uint32_t consumed_packet_count = 0;
    uint8_t distance_valid = 0;
    float measured_distance;
    float scan_roll = 0.0f;
    float scan_pitch = 0.0f;
    int8_t scan_direction = 1;
    WirelessControlData_t wireless_data;
    WirelessControlMode_t control_mode = WIRELESS_MODE_HANDHELD;
    WirelessControlMode_t previous_mode = WIRELESS_MODE_HANDHELD;

    SystemCoreClockUpdate();
    Delay_Init((uint8_t)(SystemCoreClock / 1000000U));
    DWT_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    CarMotor_Init();
    Voice_Init();
    TFT_Init();
    TFT_DrawString(8, 8, "GIMBAL " FIRMWARE_VERSION, TFT_CYAN, TFT_BLACK);
    TFT_FillRect(0, 28, TFT_WIDTH, 2, TFT_DARK_GRAY);
    TFT_DrawString(8, 40, "ROLL :", TFT_LIGHT_GRAY, TFT_BLACK);
    TFT_DrawString(8, 64, "PITCH:", TFT_LIGHT_GRAY, TFT_BLACK);
    TFT_DrawString(8, 88, "DIST :", TFT_LIGHT_GRAY, TFT_BLACK);
    TFT_DrawString(8, 112, "LINK :", TFT_LIGHT_GRAY, TFT_BLACK);
    TFT_DrawString(64, 112, "LOST", TFT_RED, TFT_BLACK);

    WirelessControl_Init();
    GasSensor_Init();

    Servo_PWM_Init();
    HC_SR04_Init();
    Buzzer_Init();
    WS2812_Init();
    WheelEncoder_Init();
    RouteLog_Init();

    TIM_SetCompare2(TIM2, 1500);
    TIM_SetCompare2(TIM3, PitchToPulse(0.0f));

    while (1)
    {
        /* 气体模块查询使用中断+状态机，不在这里等待串口响应。 */
        GasSensor_Poll();

        /* === 读取姿态 === */
        WirelessControl_Poll();
        control_mode = WirelessControl_GetMode();
        have_wireless = WirelessControl_GetLatest(&wireless_data);
        CarMotor_Service(control_mode, have_wireless ? &wireless_data : 0,
                         WirelessControl_ConsumeFault());
        {
            EncoderFrame journal_sample;
            WheelEncoder_Snapshot(&journal_sample,0);
            RouteLog_Sample(&journal_sample);
        }
        if (control_mode != previous_mode)
        {
            previous_mode = control_mode;
            if (control_mode == WIRELESS_MODE_SCAN_ALL)
                Voice_Request(VOICE_FILE_SCAN_ALL);
            scan_roll = 0.0f;
            scan_pitch = (control_mode == WIRELESS_MODE_SCAN_ALL) ? -30.0f : 0.0f;
            scan_direction = 1;
            last_scan_step_cycles = DWT->CYCCNT;
            if (control_mode != WIRELESS_MODE_HANDHELD &&
                control_mode != WIRELESS_MODE_CAR)
            {
                S1_Angle = scan_roll;
                S2_Angle = scan_pitch;
                S1_Filtered = scan_roll;
                S2_Filtered = scan_pitch;
                TIM_SetCompare2(TIM2, AngleToPulse(S1_Filtered));
                TIM_SetCompare2(TIM3, PitchToPulse(S2_Filtered));
                distance_valid = 0;
                buzzer_on = 0;
            }
        }
        if (have_wireless && wireless_data.ready)
        {
            link_was_ok = 1;

            if (control_mode == WIRELESS_MODE_HANDHELD &&
                (!have_control_packet ||
                 wireless_data.packet_count != consumed_packet_count))
            {
                consumed_packet_count = wireless_data.packet_count;
                have_control_packet = 1;

                S1_Angle = HANDHELD_ANGLE_GAIN * wireless_data.roll;
                S2_Angle = HANDHELD_ANGLE_GAIN *
                           (wireless_data.pitch - HANDHELD_PITCH_ZERO_DEG);
                S1_Filtered = S1_Angle * 0.6f + S1_Filtered * 0.4f;
                S2_Filtered = S2_Angle * 0.6f + S2_Filtered * 0.4f;

                TIM_SetCompare2(TIM2, AngleToPulse(S1_Filtered));
                TIM_SetCompare2(TIM3, PitchToPulse(S2_Filtered));
            }
        }
        else if (link_was_ok)
        {
            link_was_ok = 0;
            have_control_packet = 0;
        }

        /* === TFT显示角度 === */

        /* === 超声波测距：限制触发周期，自动扫描时等待舵机稳定 === */
        if (Scan_Update(control_mode, &scan_roll, &scan_pitch,
                        &scan_direction, &last_scan_step_cycles))
        {
            S1_Angle = scan_roll;
            S2_Angle = scan_pitch;
            S1_Filtered = S1_Angle * 0.6f + S1_Filtered * 0.4f;
            S2_Filtered = S2_Angle * 0.6f + S2_Filtered * 0.4f;
            TIM_SetCompare2(TIM2, AngleToPulse(S1_Filtered));
            TIM_SetCompare2(TIM3, PitchToPulse(S2_Filtered));
            distance_valid = 0;
            buzzer_on = 0;
        }

        if ((uint32_t)(DWT->CYCCNT - last_sonic_cycles) >=
                (SystemCoreClock / 1000U) * SONIC_PERIOD_MS &&
            (control_mode == WIRELESS_MODE_HANDHELD ||
             (uint32_t)(DWT->CYCCNT - last_scan_step_cycles) >=
                (SystemCoreClock / 1000U) * SCAN_SETTLE_MS))
        {
            last_sonic_cycles = DWT->CYCCNT;
            measured_distance = HC_SR04_GetDistance();

            if (measured_distance >= 2.0f && measured_distance <= 400.0f)
            {
                distance = measured_distance;
                distance_valid = 1;
            }
            else
            {
                distance = -1.0f;
                distance_valid = 0;
            }

            /* 显示距离 */
            /* 距离<20cm → 蜂鸣器间歇响 */
            if (distance_valid && distance < 20.0f)
                buzzer_on = 1;
            else
                buzzer_on = 0;
        }

        /* === 蜂鸣器: 有障碍物就响 === */
        if (buzzer_on)
            BUZZ_ON();
        else
            BUZZ_OFF();

        /* === DY-SV17F：只在进入预警/危险距离区域时触发一次 === */
        VoiceAlarm_Update(distance_valid, distance);
        Voice_Service();

        /* === WS2812灯带：所有控制模式只按障碍物距离显示颜色 === */
        StatusLed_Service(distance_valid, distance);

        /* 每80ms把真实姿态、测距和链路状态回传给底座ESP网页接口。 */
        if ((uint32_t)(DWT->CYCCNT - last_telemetry_cycles) >=
            (SystemCoreClock / 1000U) * 80U)
        {
            int16_t roll_tenths = 0;
            int16_t pitch_tenths = 0;
            int16_t distance_tenths = -10;
            int16_t wheel_pwm[4];
            uint8_t car_state;
            EncoderFrame encoder;

            last_telemetry_cycles = DWT->CYCCNT;
            CarMotor_GetStatus(&car_state, wheel_pwm);
            WirelessControl_SendVehicle(telemetry_sequence, (uint8_t)control_mode,
                                         car_state, wheel_pwm);
            if (have_control_packet || control_mode != WIRELESS_MODE_HANDHELD)
            {
                /* 网页雷达使用超声波探头的舵机指令角度，而不是手持端原始角度。 */
                roll_tenths = (int16_t)(S1_Filtered * 10.0f);
                pitch_tenths = (int16_t)(-S2_Filtered * 10.0f);
            }
            if (distance_valid)
                distance_tenths = (int16_t)(distance * 10.0f);

            WirelessControl_SendTelemetry(telemetry_sequence++,
                                           roll_tenths,
                                           pitch_tenths,
                                           distance_tenths,
                                           distance_valid,
                                           (uint8_t)(link_was_ok ||
                                                     control_mode != WIRELESS_MODE_HANDHELD));
            WheelEncoder_Snapshot(&encoder,telemetry_sequence);
            WirelessControl_SendEncoder(&encoder);
        }

        /* S1轮速诊断单独200ms发送，不改变E1/超声波80ms周期。 */
        if ((uint32_t)(DWT->CYCCNT - last_wheel_speed_cycles) >=
            (SystemCoreClock / 1000U) * 200U)
        {
            WheelSpeedFrame speed_frame;
            last_wheel_speed_cycles = DWT->CYCCNT;
            CarMotor_GetWheelSpeed(&speed_frame,wheel_speed_sequence++);
            WirelessControl_SendWheelSpeed(&speed_frame);
        }

        /* 每200ms发送一个气体通道，四个通道约800ms完整刷新一次。 */
        if ((uint32_t)(DWT->CYCCNT - last_gas_telemetry_cycles) >=
            (SystemCoreClock / 1000U) * 200U)
        {
            GasSensorReading_t gas;

            last_gas_telemetry_cycles = DWT->CYCCNT;
            if (GasSensor_GetReading(gas_telemetry_channel, &gas))
            {
                WirelessControl_SendGasTelemetry(
                    gas_telemetry_sequence++, gas_telemetry_channel,
                    gas.online, gas.valid, gas.concentration_raw, gas.unit_code,
                    gas.decimals, gas.status, gas.gas_type);
            }
            gas_telemetry_channel = (uint8_t)((gas_telemetry_channel + 1U) %
                                               GAS_SENSOR_CHANNEL_COUNT);
        }

        if ((uint32_t)(DWT->CYCCNT - last_display_cycles) >=
            (SystemCoreClock / 1000U) * 200U)
        {
            uint8_t car_state;
            int16_t wheel_pwm[4];
            uint8_t hold_reason;
            uint8_t pose_live;
            const char *car_label;
            const char *labels[] = {"CAR:OFF  ", "CAR:OFF  ", "CAR:LEVEL",
                                    "CAR:READY", "CAR:RUN  ", "CAR:LOST ", "CAR:FAULT"};
            last_display_cycles = DWT->CYCCNT;
            CarMotor_GetStatus(&car_state, wheel_pwm);
            hold_reason = CarMotor_GetHoldReason();
            pose_live = (uint8_t)(have_wireless && wireless_data.ready);
            if (pose_live && control_mode == WIRELESS_MODE_CAR)
                pose_live = (uint8_t)(wireless_data.protocol == 1U &&
                    wireless_data.requested_mode == 1U &&
                    (uint32_t)(DWT->CYCCNT - wireless_data.arrival_cycles) <=
                        (SystemCoreClock / 1000U) * CAR_LINK_TIMEOUT_MS);
            car_label = labels[car_state <= CAR_FAULT ? car_state : CAR_FAULT];
            if (car_state == CAR_NEUTRAL)
            {
                if (hold_reason == CAR_HOLD_INPUT) car_label = "CAR:INPUT";
                else if (hold_reason == CAR_HOLD_TILT) car_label = "CAR:TILT ";
                else if (hold_reason == CAR_HOLD_RECOVER) car_label = "CAR:CHECK";
            }
            TFT_DrawString(8, 136, car_label,
                           car_state == CAR_MOVING ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);

            if (control_mode != WIRELESS_MODE_HANDHELD &&
                control_mode != WIRELESS_MODE_CAR)
            {
                TFT_DrawSigned3(64, 40, (int32_t)S1_Filtered,
                                TFT_YELLOW, TFT_BLACK);
                TFT_DrawSigned3(64, 64, (int32_t)(-S2_Filtered),
                                TFT_YELLOW, TFT_BLACK);
            }
            else if (pose_live)
            {
                TFT_DrawSigned3(64, 40, (int32_t)wireless_data.roll,
                                TFT_YELLOW, TFT_BLACK);
                TFT_DrawSigned3(64, 64, (int32_t)wireless_data.pitch,
                                TFT_YELLOW, TFT_BLACK);
            }
            else
            {
                TFT_DrawString(64, 40, "----", TFT_DARK_GRAY, TFT_BLACK);
                TFT_DrawString(64, 64, "----", TFT_DARK_GRAY, TFT_BLACK);
            }

            if (distance_valid)
            {
                TFT_DrawUnsigned3(64, 88, (uint32_t)distance,
                                  TFT_GREEN, TFT_BLACK);
                TFT_DrawString(88, 88, "CM", TFT_GREEN, TFT_BLACK);
            }
            else
            {
                TFT_DrawString(64, 88, "--- CM", TFT_DARK_GRAY, TFT_BLACK);
            }

            if (control_mode == WIRELESS_MODE_CAR ? pose_live : link_was_ok)
                TFT_DrawString(64, 112, "OK  ", TFT_GREEN, TFT_BLACK);
            else
                TFT_DrawString(64, 112, "LOST", TFT_RED, TFT_BLACK);
        }

        /* Low-priority read-only replay never waits for UART or the browser. */
        WirelessControl_SendLog();
        /* === 角度→舵机 === */
    }
}
