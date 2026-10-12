# VSCode 打开 C 项目耗尽内存：诊断与处理

原诊断日期：2026-09-15；项目路径说明于 2026-09-20 更新。用户级设置与当时观测属于历史记录，本次没有重新修改全局配置。

## 已确认的原因

这不是当前 STM32 工程本身太大：检查时整个目录约 41 MB、约 290 个文件（包含编译产物）。

VSCode 实际恢复的是一个多目录工作区，包含 epass、lagarain、e_competiton、jetson、epass_board。仅打开 balance_car 中的一个文件不等于将工作区切换为 balance_car；语言扩展可能同时处理工作区其他目录。

内核日志在 09:42:50 显示一个 cpptools 和八个 cpptools-srv2，RSS 合计约 7580 MiB（7.4 GiB）。机器物理内存约 15.2 GiB，4 GiB swap 几乎耗尽；日志明确记录 cpptools invoked oom-killer，VSCode 和 ChatGPT 进程被内核杀掉。这里统计的是驻留内存，而非 Electron 很大的虚拟地址空间。

同时，Pylance 日志显示扫描 epass/buildroot/output 下的源码、工具链和软链接，诊断时该 Python 服务的 RSS 约 1.2 GB。C/C++ 扩展与 Python 扩展有各自的扫描机制，files.watcherExclude 不能代替语言服务的排除规则。

旧 epass 配置虽然关闭了当地 IntelliSense，但没有给其他工作区的 C/C++ 服务设置统一并发预算。旧的 intelliSenseMemoryLimit=2048 是单个 IntelliSense 进程限制，不是整个扩展总限制，也不限制其他扩展。

Windows 上没有做同版本、同扩展、同目录、同内存的对照，因此不能把差异直接归因于 Linux。当前 Linux 上已有明确的多进程内存和多目录扫描证据。

## 已应用的修改

### 当前用户 VSCode 设置

路径：`/home/mechcid/.config/Code/User/settings.json`。

- C_Cpp.maxConcurrentThreads=2。
- C_Cpp.maxMemory=1024、intelliSense.maxMemory=768、intelliSenseMemoryLimit=512（单位 MB）。
- 查找引用和代码分析各限制为一个并发线程，预算 512 MB。
- 关闭自动代码分析，后台工作区解析优先级设为 low。
- Python 关闭全局索引、仅诊断打开的文件，排除 Buildroot、build/install/log 等产物目录。
- Python Node 堆预算设置为 1536 MB，重启语言服务后生效。

这些设置是语言服务的缓存/并发调节阈值，**不是操作系统强制的 VSCode 总内存硬上限**；Python 的 Node 堆也不等于进程总 RSS。大文件仍可能触发语言服务重启，多个窗口和其他扩展仍会增加总内存。

影响：Python 未打开文件的自动导入建议可能减少，后台分析更保守；C/C++ 分析可能变慢。没有卸载扩展或修改系统 swap。

### 当前 STM32 项目

路径：`.vscode/settings.json`。

- 保留 IntelliSense，使用实际 ARM 编译器与 `build/Debug/compile_commands.json`。
- 限定浏览路径为 Core、module、Drivers，保持 HAL/CMSIS 源码可见。
- 排除 build 的 C/C++ 后台扫描；旧测试和备份目录已按用户要求删除。
- 排除产物的文件监听和全局搜索，没有隐藏核心源码。
- 本项目 Python 采用 light 模式。
- 禁止 CMake 在打开/修改配置时自动触发配置，手动构建入口保持不变。

编译数据库包含 MPU6050、OLED 和 UART 驱动；C/C++ 设置键和枚举已与本机扩展 1.34.4 的配置声明核对。

## 验证状态

已完成设置写入和静态校验。尝试打开时 VSCode 又恢复了旧多目录窗口，并把 C 文件送入旧窗口，因此不能用该窗口证明独立项目修复效果。

最后一次打开 C 文件的操作被用户中断。**尚未完成修复后持续内存监测和代码跳转实测，不能宣称已彻底解决。**

建议在 VSCode 中使用“文件 → 打开文件夹”，选择 `/home/mechcid/work/balance_car`，确认标题为 balance_car，而不是“无标题（工作区）”。重新加载窗口后打开 module/base/src/mpu6050.c，再观察任务管理器中的 cpptools 进程和内存，并测试 HAL_I2C_Mem_Read 的定义跳转。

当前配置未恢复 epass 文件夹中原来关闭的 IntelliSense；Linux 内核的精确跳转需要为该内核单独选择索引范围/编译数据库，不能把整个 Buildroot 根目录不加区分地交给扩展扫描。

## 当前配置与恢复边界

旧备份已按 2026-09-20 的清理要求删除，不再提供恢复旧用户设置/多目录工作区的归档。项目级限制可在 `.vscode/settings.json` 中逐项调整；这不影响固件源码。用户级配置与项目级配置作用范围不同，勿将删除本项目配置等同于恢复全局设置。

新的图形化构建入口见 [VSCode编译烧录调试.md](VSCode编译烧录调试.md)。

## 参考

[Microsoft C/C++ 配置说明](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)说明编译数据库和头文件搜索路径如何影响 IntelliSense；本次具体的预算设置语义另以已安装扩展的 package.json/package.nls.json 为准。
