# SysID

基于 Motor 接口的单轴闭环系统辨识：四电平伪随机力矩＋弱 PD 回中，在 Ozone 修改一个命令启动。记录后通过积分最小二乘估计转动惯量 J、粘性阻尼 b；不预设任何机器的 J、零点或机械限位。

## 按 XRobot 流程接入（1.0.1）

模块入口是 `SysID.hpp` 中的 `class SysID`，manifest 为 `standalone: true`。
在 YAML 实例化后，`xrobot setup` / `xrobot gen` 自动生成电机、IMU、AHRS 和辨识器的构造代码；SysID 自己创建 2 ms 控制线程。无需手写 app_main 调用、Poll 循环或单独的启动头文件。

1. 在 `Modules/sources.yaml` 的 sources 列表增加：

```yaml
- url: https://raw.githubusercontent.com/MonaFan1/SysID/main/index.yaml
  priority: 0
```

2. 在 `Modules/modules.yaml` 的 modules 列表增加 `MonaFan1/SysID@main`。入口 manifest 的 depends 自动拉取 Motor、RMMotor、DMMotor、BMI088、MadgwickAHRS。
3. 完整应用配置参考 [examples/small_yaw.yaml](examples/small_yaw.yaml)：先实例化 BMI088、AHRS 和电机，再配置 SysID，motor 引用前面已经创建的电机，can_bus 使用该电机的实际 CAN。复制示例到 BSP 的 User/xrobot.yaml 后执行：

```powershell
xrobot setup
xrobot format
xrobot gen -c User/xrobot.yaml
cmake --preset debug
cmake --build --preset debug
```

SysID 的参数均通过构造参数 `param` 配置，示例包含 CAN 反馈 ID、力矩反馈符号、IMU 话题、gyro 分量/符号、pitch 零位、位置边界、激励/PD/采集参数及线程栈大小。示例里的 GM6020、CAN1、0x205、方向和限位只适用于该示例机器，移植时按实物配置，不是模块默认机器参数。模块不写入固定惯量。

**电机独占**：这个 YAML 入口持续负责绑定电机的 Update/Control/Relax，空闲发送零输出。不能同时让 Gimbal 等其他控制器使用同一电机。RMMotor 同组其他电机仍须参与各自的驱动拼包。该对象及其硬件依赖保持静态生命周期。

位置使用电机编码器，PD 与拟合速度使用有符号的 IMU 分量，电机速度另外记录对照。统一单位 rad、rad/s、N·m；固定底座、大 yaw 和 pitch，pitch 接近校准零位，安装轴对齐且 roll 固定。任意姿态下的 gyro.z 不等于 Euler yaw 的导数。结果是这些机械条件下的单轴等效参数。

公开版 DM 已对反向配置的反馈力矩取反，feedback_torque_sign 通常 +1；当前公开版 RM 未翻转反向配置的反馈力矩，reverse=true 时设为 -1。实际 gyro 分量/符号指 IMU rotation 之后的坐标，应满足编码器 position 增大时使用的角速度为正。

低层 `MotorIdentifier.hpp` 保留手动 Bind/Poll 接口，供已有控制循环的高级集成使用；常规 XRobot 用户使用上面的 SysID 入口。

## Ozone 启动与参数

YAML 实例 id 建议为 g_sysid。调试器若无法直接按短名称找到它，可在生成的 XRobotMain 中查看同名静态对象。上电线程自动启动，ready_=true 表示绑定完成；实验仍空闲，不自动开始。先固定底座/大 yaw，并将 pitch 保持在校准零位、yaw 放在有活动余量的位置，再改参数，最后将 command_ 改为 1。参数在启动时锁存，中途修改下次才生效。

| Watch 变量（前缀 g_sysid.） | 默认 | 含义 |
| --- | --- | --- |
| command_ | 0 | 1 启动 PRBS＋PD；0 停止；2 零力矩手动刻度检查 |
| amplitude_ | 0.10 | 激励基数 N·m，四档为 ±0.75A、±0.25A |
| kp_ / kd_ | 2 / 0.12 | 相对启动位置的回中 PD |
| torque_limit_ | 0.20 | 总力矩上限 N·m，范围 (0,2] |
| max_travel_ | 0.06 | 相对起点单侧最大行程 rad，范围 (0,0.5] |
| min_position_ / max_position_ | ±1000000 | 绝对编码边界，默认未设置，按机构填写 |
| duration_s_ | 14 | 总时长 s，范围 (2,14]，首尾各 1 s 只有 PD |
| hold_s_ | 0.10 | 档位保持时间 s，范围 [0.02,1] |
| seed_ | 0xACE1 | 16 位非零种子；换种子复测 |

默认配置需按机构调整；若不能产生持续运动，无法可靠辨识 J、b。command_=2 仅供手动检查刻度，全程零力矩，不用于 J、b 辨识。

正常运行约 14 s 自动退出并将 command_ 清零，缓冲区冻结。中途 command_=0 也会退出。试验期间暂停调试会破坏时序，需重新开始；启动前的暂停不计入试验。50 ms 无反馈视为失联，开始前最多等待约 5 s；温度达到 75℃、超速、越界也会退出，最大允许控制周期 20 ms。退出时撤销输出并调用 Relax。

status_：0 等待/手动停止，1 运行，2 完成，3 参数或采样非法，4 反馈丢失，5 超速，6 位置跳变或绝对边界越界，7 缓冲区满，8 单位未确认，9 旧恒速模式未停稳，10 相对行程越界，11 过温，12 命令非法，13 IMU 未接入或失联，14 pitch 不在校准零位附近或试验中变化超过 0.05 rad。

theta_ 是相对起点位置；omega_ 是位置窗口斜率诊断值。PD 和新版日志使用带方向符号的 IMU 角速度。omega_imu_ 可直接观察，omega_motor_ 为电机速度，pitch_imu_ 为相对校准零位的 pitch。通用默认速度上限为 3 rad/s，IMU 与电机速度均检查。


### IMU 接线与配置说明

YAML 参数 gyro_topic_name、euler_topic_name、gyro_axis（0=x、1=y、2=z）、gyro_sign、pitch_zero 分别指定话题、分量、方向和 pitch 零位。分量指 BMI088 发布话题已经执行 rotation 变换之后的坐标，不能把安装变换重复做一遍。上面的示例选择 -gyro.z()；实际机构必须验证正方向：编码器 position 增大时，omega_imu_ 也应为正。

gyro 话题类型是 Eigen::Matrix<float,3,1>、单位 rad/s；Euler 是 LibXR::EulerAngle<float>、单位 rad。ImuFeedback 在 Poll 内自己取话题，不用 Gimbal 转发。gyro 超过 20 ms、Euler 超过 50 ms 未更新就退出，且不会回退到电机速度。这里的新鲜度按本模块读取新话题的时刻计算，不是传感器硬件采样时间。pitch 相对 pitch_zero 指定零位必须在 ±0.05 rad，运行中变化也不能超过 0.05 rad。

BMI088/AHRS 依赖只负责数据提供；使用其他 IMU 时，可以提供相同类型和单位的话题，按实际提供者调整 manifest。零力矩手动刻度检查 command_=2 不要求 IMU。

## 导出与拟合

确认 done_=true，正常完成为 status_=2。Ozone Save Memory Data 起点为 &g_sysid.buf_[0] 解析出的地址，Size 按十六进制填写 **C008**，即 **49160 字节**。不导出整个对象。

0xC000 字节为 1536 个 32 字节样本，最后 8 字节为写入索引、有效数量。样本依次为：时间、上一记录区间总命令力矩均值、相对位置、IMU 端点角速度、电机端点角速度、反馈力矩区间均值、阶段、标志（六个 float32、两个 uint32，小端）。IMU PRBS 阶段为 400/401/403；旧电机速度版本的 300/301/303 数据不被新版脚本接受。与 Double Yaw 及旧模式格式不同。文件不含终止状态，须另看 Ozone 的 status_。

```powershell
python -m pip install numpy scipy
python ./identify.py ./capture.bin --fit
```

输出同名 CSV、metadata.json、fit.json。默认同时估计未知 J、b；已有独立可信 J 时可额外传 --fixed-j 数值，由用户提供，不写死在模块里。

拟合模型：tau = J*domega/dt + b*omega + fc*sign(omega) + k*q + offset。对 40/80/160 ms 区间积分，避免直接差分速度；比较命令、反馈力矩两组结果。使用激励与 PD 合成、限幅后的总输入，所以不再从结果 b 中减去 kd。

附加摩擦、位置和偏置项用于减轻遗漏项影响，并非保证每项均可独立辨识。脚本剔除限幅、低速和跨正反转的窗口，并报告条件数、边界、末段留出误差。不同窗口、输入和种子结果应接近。闭环相关性、噪声和延迟仍可能造成偏差，需换种子复测。反馈力矩是基于电流/协议量程的估计，不是独立传感器。

旧 MATLAB 流程本身也使用最小二乘/模型拟合。新版主要改进在区间积分、总力矩输入和窗口检查，并不是“最小二乘对另一种完全不同算法”。

## 文件

- SysID.hpp：XRobot/YAML 入口、参数绑定和自动线程。
- MotorIdentifier.hpp：Motor/CAN/IMU 适配与 Ozone 启停。
- ImuFeedback.hpp：独立订阅角速度和姿态话题、检查新鲜度。
- MotorSystemID.hpp：无硬件依赖的激励与采集核心，保留旧检查模式。
- identify.py：新版 PRBS 解码及离线积分拟合。
- tests：模拟采集与拟合验证。


## 本机测试

在仓库根目录运行，需要 Python、numpy/scipy 和 PATH 中的 g++。可以执行 `python tests/run_tests.py` 完成所有模拟测试；也可逐项运行下面命令。使用临时目录存放模拟数据和可执行文件，不需要连接电机：

```powershell
New-Item -ItemType Directory -Force $env:TEMP/sysid-test
g++ -std=c++17 -O2 -I . ./tests/motor_sysid_test.cpp -o $env:TEMP/sysid-test/core.exe
& $env:TEMP/sysid-test/core.exe $env:TEMP/sysid-test
g++ -std=c++17 -O2 -I ./tests/stubs -I . ./tests/motor_identifier_test.cpp -o $env:TEMP/sysid-test/adapter.exe
& $env:TEMP/sysid-test/adapter.exe
python ./tests/test_identify.py $env:TEMP/sysid-test
```

测试中的模拟 J、b 仅存在测试文件中，不参与固件或真实数据的默认拟合。
