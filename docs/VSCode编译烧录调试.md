# VSCode 中编译、烧录、调试 STM32

更新：2026-09-29。适用：本机 Linux、STM32F103C8T6、CMSIS-DAP、当前工程。

## 1. 直接怎么用

先用“文件 → 打开文件夹”打开 `/home/mechcid/work/balance_car`，避免恢复包含 Buildroot/Linux 源码的巨大多目录工作区。保存修改后操作。

| 目的 | 图形界面 / 快捷键 | 实际结果 |
|---|---|---|
| 编译并烧录运行 | **Ctrl+Shift+B**，或“终端 → 运行生成任务” | 配置、编译、烧录、校验、复位运行 |
| 只编译 | “终端 → 运行任务 → STM32: 编译 Debug” | 不操作板卡，生成 ELF/BIN |
| 启动调试 | 左侧“运行和调试”选择 `STM32: 编译并调试 (CMSIS-DAP)`，点绿色三角或 **F5** | 先编译，再烧录，停在 main |
| 继续执行 | 调试时 F5 | 运行到下一个断点 |
| 单步越过 / 进入函数 | F10 / F11 | 查看下一步运行结果 |
| 停止调试 | Shift+F5 / 红色停止按钮 | 结束调试会话；要确保独立运行可重新执行编译并烧录任务 |

**本工程的 Ctrl+Shift+B 会写入板卡 Flash**，不仅是编译。没有连接 CMSIS-DAP 时请选“只编译”任务。

开始调试后点击代码行号左边设置断点。例如在 `module/src/sensor_app.c` 的 `status.reads++` 处停住，展开变量查看 `corrected_sample.ax_g/ay_g/az_g`。先按 F5 走过 `main` 的启动断点，OLED 才会初始化；CPU 停住时屏幕数字停止更新是正常的。

编译错误会出现在“问题”面板与任务终端；遇到失败请修复后重试，不要选择用旧固件继续调试。结束一个调试会话后再烧录，避免两个程序同时占用 CMSIS-DAP。

## 2. 一键操作背后发生了什么

```text
你的 .c/.h
   ↓ CMake：读取 CMakeLists、选择工具链和源码，生成 Ninja 构建计划
   ↓ Ninja：按依赖顺序调用 arm-none-eabi-gcc
   ↓ 编译/汇编：各文件变成 .o
   ↓ 链接：按 STM32F103XX_FLASH.ld 放置 Flash、RAM、向量表等
balance_car.elf（机器码 + 地址 + Debug 符号）
   ├─ objcopy → balance_car.bin（裸二进制）
   └─ OpenOCD → CMSIS-DAP → SWD → STM32 Flash，校验后复位
```

VSCode 是编辑与操作入口，CMake 管理构建，GCC 生成 ARM 指令，OpenOCD 控制调试器。仅配置 IntelliSense 不会自动产生可烧录的程序；仅安装 C/C++ 扩展也不会代替交叉编译器。

调试另外多一条双向控制链：

```text
VSCode 运行和调试界面
   ↕ Cortex-Debug 扩展
GDB（本机 gdb-multiarch）
   ↕ 本机调试服务连接
OpenOCD ↔ CMSIS-DAP ↔ STM32
```

GDB 用 ELF 中的调试符号把机器地址对应到源码行和变量，OpenOCD 负责实际暂停 CPU、设置硬件断点、读写内存。程序运行在 STM32 上，而不是电脑上。Debug 用 `-O0 -g3` 便于逐行调试；Release 用 `-Os -g0` 缩小体积，不适合作为本配置的源码调试目标。

## 3. 本次配置了哪些文件

| 文件 | 本次作用 |
|---|---|
| `.vscode/tasks.json` | 三个任务：配置 Debug、编译 Debug、编译并烧录；以 dependsOn 串联；最后一个是默认生成任务 |
| `.vscode/launch.json` | Cortex-Debug + OpenOCD + CMSIS-DAP；指定 ELF、GDB、芯片、SWD 和停在 main；preLaunchTask 先编译 |
| `.vscode/settings.json` | 把旧 UART 索引路径迁移到 module，继续使用真实 ARM 编译数据库；保持内存限制与关闭自动配置 |
| `.vscode/extensions.json` | 推荐 C/C++、CMake Tools、Cortex-Debug 三个扩展 |
| `CMakeLists.txt` | 登记 module/src 的模块实现文件，添加 module/inc，保持 ELF 转 BIN |
| `CMakePresets.json` | 沿用现有 Debug/Release 预设，任务直接调用它们 |
| `cmake/gcc-arm-none-eabi.cmake` | 沿用现有 ARM 工具链、链接脚本与编译选项 |

编译任务使用 `--parallel 2`，限制同时编译数量；这是编译阶段的限制，与 IntelliSense 后台索引的内存控制是两回事。没有修改本机全局快捷键、安装扩展、修改系统服务或 sudo 配置。

新建 `.h` 放入 `module/inc` 后通常不必改 CMake；新建 `.c` 放入 `module/src` 后需加入顶层 `target_sources`。采用明确文件列表，避免临时文件被自动编译。

### 3.1 settings.json：让编辑器正确理解工程

**与你观察到的“头文件能解析、能跳转”最直接相关的是 `.vscode/settings.json`，以及它指向的 `build/Debug/compile_commands.json`。** 根据当前配置，这两处让 C/C++ 扩展获得了 ARM 编译器、头文件目录和条件编译宏。此前没有逐项开关做对照实验，所以不能把恢复归因于某一个开关。

编译器知道如何编译，不代表编辑器自动知道如何解析。比如 `sensor_app.c` 中写了 `#include "mpu6050.h"`，编辑器需要知道应到 `module/inc` 查找；处理 STM32 头文件时，还需要知道启用了 `STM32F103xB` 和 `USE_HAL_DRIVER`。这些条件不正确时，可能出现找不到头文件、类型未定义、条件编译分支变灰或无法准确跳转。

本项目关键配置如下（表中前缀 `C_Cpp.default.` 已省略）：

| 配置 | 当前值/作用 |
|---|---|
| `compilerPath` | `/usr/bin/arm-none-eabi-g++`；为未加入构建的新 C++ 文件查询 ARM C++ 标准库路径和预定义宏 |
| `intelliSenseMode` | `linux-gcc-arm`；指定目标架构对应的解析模式，不能把 STM32 当电脑的 x86 程序理解 |
| `compileCommands` | 指向 `build/Debug/compile_commands.json`；按每个源文件的真实编译命令配置解析 |
| `includePath` | 列出 `Core/Inc`、`module/inc`、HAL、CMSIS；作为基础配置中的头文件搜索目录 |
| `defines` | `USE_HAL_DRIVER`、`STM32F103xB`；基础配置中的条件编译宏 |
| `cStandard` | `c11`；基础配置采用的 C 语言标准 |
| `browse.path` | 限定符号浏览的范围为 Core、module、Drivers；与精确语义解析使用的编译参数作用不同 |

`C_Cpp.intelliSenseEngine=default` 开启正常的语义分析。`C_Cpp.intelliSenseMemoryLimit`、排除规则和 Python 轻量设置用于降低后台负担；它们并不提供头文件路径，也不是 VSCode 总内存的硬限制。`files.watcherExclude` 控制文件监听，`search.exclude` 控制搜索范围，二者不能代替 `includePath`。本项目没有因此隐藏模块源码。

**compile_commands.json 为什么重要？** 顶层 CMake 启用了 `CMAKE_EXPORT_COMPILE_COMMANDS`，配置阶段会自动生成这份“每个 .c 应该怎样编译”的清单，包含编译器、`-I` 目录、`-D` 宏和其他选项。C/C++ 扩展找到当前源文件的匹配记录时，会以该记录配置 IntelliSense；没有匹配记录时使用基础配置。头文件通常根据包含它的源文件上下文解析，不需要给每个 `.h` 单独编译。

```text
CMakeLists.txt：登记模块源码、头文件目录
   ↓ cmake --preset Debug
build/Debug/compile_commands.json：生成真实编译参数清单
   ↑ .vscode/settings.json 中的 compileCommands 指向它
C/C++ 扩展 → 解析 #include、条件编译和类型 → 补全、诊断、跳转
```

因此不要手改生成的 `compile_commands.json`。新增模块时，应修改 CMake 并重新执行配置/编译任务；只修改编辑器的 `includePath`，并不会替构建系统添加 `-I` 参数。

一些教程使用 `.vscode/c_cpp_properties.json`。它是 C/C++ 扩展专用的配置文件；**本工程用 settings.json 中的 `C_Cpp.default.*` 和编译数据库完成配置，所以目前不需要额外建立第五个 JSON**。如果以后新增该文件，其中显式配置的字段可能覆盖相应默认值，维护时要避免两处不一致。相关优先关系见 [C/C++ 官方配置说明](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)。

作用范围也要区分：项目的 `.vscode/settings.json` 随项目保存；Linux 用户设置通常在 `~/.config/Code/User/settings.json`，会影响多个项目。本项目的头文件路径写在项目配置里。之前为内存问题调整过用户级设置，历史说明见 [VSCode内存问题.md](VSCode内存问题.md)。同一可覆盖设置通常由更具体的工作区/文件夹配置覆盖用户默认值，参见 [VSCode 设置作用范围](https://code.visualstudio.com/docs/configure/settings)。

### 新增 C++ 模块的编辑器配置（2026-09-21）

电机相关 `.hpp` 暂未接入 CMake，没有对应源文件的编译数据库记录，因此补齐基础解析配置：默认编译器使用 `arm-none-eabi-g++`，目标参数为 `-mcpu=cortex-m3 -mthumb`，C++ 标准为 C++17，并明确将 `.hpp/.cpp` 识别为 C++。C 标准仍为 C11；已有 C 文件继续优先使用编译数据库中的实际 GCC 命令。这不会将现有 C 固件改为 C++ 构建。

已使用这些基础配置对五个电机相关头文件执行 ARM C++17 语法检查，通过。尚未在 VSCode 窗口核对红线是否全部消失；若仍显示旧诊断，可执行命令面板中的 `C/C++: Reset IntelliSense Database`，必要时再执行 `Developer: Reload Window`。没有关闭错误提示或手工添加本机 x86 标准库目录。

### 3.2 tasks.json：把操作命令变成菜单和快捷键

它告诉 VSCode：“选择一个任务时执行什么程序、带哪些参数、先执行哪个任务。”本工程中：

```text
STM32: 编译并烧录
   dependsOn → STM32: 编译 Debug
                  dependsOn → STM32: 配置 Debug

实际执行顺序：配置 → 编译 → 烧录校验并复位
```

- `label`：菜单中显示的任务名，也是其他任务引用它时使用的名字。
- `type: process`、`command`、`args`：直接启动 cmake/openocd，并传入各个参数。
- `options.cwd`：在项目根目录执行；`${workspaceFolder}` 表示当前打开的工程目录。
- `dependsOn`：先完成前置任务；无需你手动分三次操作。
- `problemMatcher: ["$gcc"]`：把 GCC 输出的文件、行号和错误提取到“问题”面板，便于点击定位。
- `group.kind: build`、`isDefault: true`：把“编译并烧录”指定为默认生成任务，所以 Ctrl+Shift+B 会执行它。这里没有重新定义键盘快捷键，而是使用 VSCode 自带快捷键调用默认任务。

它负责调用构建工具，真正“哪些源文件参与编译、链接哪些库”仍由 CMake 决定。它不直接控制头文件跳转，但配置任务生成的编译数据库会帮助编辑器解析工程。

### 3.3 launch.json：让 F5 知道如何调试这块 STM32

它描述一场调试会话：用哪个调试扩展、加载哪个固件、通过什么设备连接芯片。

| 字段 | 本工程的含义 |
|---|---|
| `type: cortex-debug` | 使用 Cortex-Debug 扩展处理嵌入式调试 |
| `request: launch` | 发起固件启动调试流程 |
| `executable` | 指向 Debug ELF；调试器从中获得机器码和源码/变量符号 |
| `servertype`、`serverpath` | 启动本机 OpenOCD 调试服务 |
| `configFiles` | 加载项目共用的 `.vscode/openocd.cfg`，选择 CMSIS-DAP、STM32F1、SWD 和 100 kHz |
| `gdbPath`、`armToolchainPath` | 指定 GDB 和 ARM 辅助工具的位置 |
| `device`、`interface` | 标识 STM32F103C8 与 SWD；OpenOCD 的目标配置由上述 configFiles 提供 |
| `preLaunchTask` | 引用 tasks.json 中的“STM32: 编译 Debug”，确保调试前先构建 |
| `runToEntryPoint: main` | 启动后运行到 main 并暂停，方便开始单步调试 |

F5 的便利来自 `launch.json` 与 `tasks.json` 的配合。这里的“跳到 main 断点”是程序运行中的调试行为；编辑代码时按 F12“转到定义”则是 C/C++ 扩展的源码解析功能，两者依赖不同配置。

### 3.4 extensions.json：列出工程推荐使用的扩展

本工程的 `recommendations` 列出三个扩展：

| 扩展 ID | 负责什么 |
|---|---|
| `ms-vscode.cpptools` | C/C++ 语义分析、补全、头文件与定义跳转等 |
| `ms-vscode.cmake-tools` | CMake 预设、配置和构建的图形化集成；本工程 tasks 也可直接调用 CMake |
| `marus25.cortex-debug` | 识别 launch.json 中的 cortex-debug 配置，提供 STM32 调试界面 |

这是一份推荐清单，方便换电脑或其他人打开工程时安装相应扩展；**文件本身不会自动安装 GCC、CMake、OpenOCD，也不会保证扩展已经安装或启用**。工具路径仍由其他配置和系统环境提供。参见 [VSCode 扩展推荐说明](https://code.visualstudio.com/docs/configure/extensions/extension-marketplace)。

### 3.5 以后遇到问题，优先看哪里

| 现象/需求 | 优先检查 |
|---|---|
| 能编译，但头文件报红、F12 无法跳转 | settings.json 的编译器和 compileCommands；编译数据库是否存在并含对应源文件；C/C++ 扩展是否启用 |
| 编辑器不报错，实际编译却找不到头文件 | CMakeLists.txt 的 target_include_directories；编辑器配置不会自动修改构建参数 |
| 新加 .c 没有参与编译 | CMakeLists.txt 的 target_sources，随后重新配置 |
| 修改 Ctrl+Shift+B 执行的动作 | tasks.json 中哪个任务被设为默认 build 任务 |
| 修改烧录命令或参数 | tasks.json 中“编译并烧录”的 command/args |
| F5 找不到 ELF/GDB/OpenOCD 或无法连板 | launch.json 的路径与目标配置、前置编译任务、CMSIS-DAP 连接 |
| 换电脑后不知道该装什么扩展 | extensions.json；再单独安装系统工具链并修正路径 |

简记：**settings 管编辑体验，tasks 管执行命令，launch 管调试会话，extensions 管扩展推荐；CMake 管真实的构建规则。**

## 4. 当前工具与路径

本机已找到 `cmake`、`ninja`、`arm-none-eabi-gcc`、`arm-none-eabi-objcopy`、`openocd`、`gdb-multiarch`，所需三个扩展已经安装。

调试配置明确使用 `/usr/bin/gdb-multiarch`、`/usr/bin/openocd` 和 `/usr/bin` 下的 ARM 工具。工程路径使用 `${workspaceFolder}`；复制到 Windows 时需要替换这些 Linux 工具路径。硬件和源码逻辑相同，操作系统上的工具位置和 USB 权限不同。

下载和调试共用 `.vscode/openocd.cfg`，内部加载安装包的 `interface/cmsis-dap.cfg`、`target/stm32f1x.cfg`，显式选择 SWD，并在目标配置之后设置 100 kHz。以后调整速度只需修改这一处。本机普通用户权限下已成功访问 CMSIS-DAP；Codex 测试时的授权是运行沙箱的设备/连接访问限制，不代表 VSCode 任务需要 sudo。不要用 sudo 启动 VSCode。

## 5. 等价命令：便于理解与排错

在项目根目录运行：

```bash
cmake --preset Debug
cmake --build --preset Debug --parallel 2
openocd -f .vscode/openocd.cfg \
  -c 'program build/Debug/balance_car.elf verify reset exit'
```

产物在 `build/Debug/`：ELF 用于烧录/调试，BIN 用于需要裸二进制的工具，MAP 用于看空间分布，`compile_commands.json` 供代码索引使用。`build/` 可删除后重建，不属于手写源码。

## 6. 已验证与边界

- 本次配置切换后，CMake 配置与 Debug 构建通过，共用的 `.vscode/openocd.cfg` 实机识别到 Cortex-M3，SWD 100 kHz。
- 同日此前已通过相同 DAP/SWD 参数完成 Debug 下载与校验（Verified OK），GDB 命中 SensorApp_Update 临时硬件断点，并完成单步与变量读取；详见 [DAP 下载调试验证](DAP下载调试验证.md)。
- 本次没有在 VSCode 窗口亲自点击 F5/Ctrl+Shift+B；验证的是配置文件和底层工具链。F5 的 runToEntryPoint 仍为 main。
- 电机停止时再调试；CPU 暂停期间软件超时保护不能执行。暂停可能扰乱传感器通信，结束测试后应复位并确认采集恢复。

如果构建成功但烧录失败，先看任务终端是否能发现 CMSIS-DAP；如果 F5 启动后屏幕不动，先看是否正停在断点；如果“找不到 ELF”，先确认打开的是项目根目录并执行编译任务。

## 7. 配置依据

- [VSCode Tasks 官方文档](https://code.visualstudio.com/docs/debugtest/tasks)：任务和依赖关系。
- [VSCode 调试配置官方文档](https://code.visualstudio.com/docs/debugtest/debugging-configuration)：launch.json 和 preLaunchTask。
- [Cortex-Debug 官方配置字段](https://github.com/Marus/cortex-debug/blob/master/debug_attributes.md)：OpenOCD、GDB 与 configFiles 等字段。
