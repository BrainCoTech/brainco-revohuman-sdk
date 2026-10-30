# RevoHuman C SDK

RevoHuman 数据手套的 C 语言库：连接手套，读取每手 21 个关节角与全指触觉，并在库内算手部正向运动学（FK）。
接口是 C99，C++ 可以直接包含。适合不带 Python 运行时的程序和嵌入式设备。

Tracker 位姿、腕部相机、MCAP 录制和 ROS 2 节点请用 Python SDK，见本仓库 [main 分支](https://github.com/BrainCoTech/brainco-revohuman-sdk/tree/main)。
两者的错误码逐值相同，C 库的本机广播可以直接接 Python SDK 的查看器。

这里只放公开头文件、最小示例和许可证。库以二进制包发布，不含源码。

```
LICENSE                     Apache-2.0 许可证全文
include/revohuman.h         手套、触觉、双手配对、套件接口
include/revohuman_fk.h      手部正向运动学
example/                    不接手套就能跑的最小示例
```

全部接口与约定写在两个头文件的注释里。

---

## 支持的硬件与系统

| 项 | 要求 |
|---|---|
| 手套 | RevoHuman P1 或 PTV1，左右手各一只，USB 接主机 |
| 架构 | x86_64 / aarch64 |
| 系统 | Linux，glibc ≥ 2.34，libstdc++ 提供 GLIBCXX ≥ 3.4.30（Ubuntu 22.04 及以上满足） |
| 权限 | 运行用户要有串口权限（通常是加入 `dialout` 组） |

## 下载与校验

发布文件在 [GitHub Release c-sdk-v0.1.2](https://github.com/BrainCoTech/brainco-revohuman-sdk/releases/tag/c-sdk-v0.1.2) 下载：

| 文件 | 内容 |
|---|---|
| `revohuman-c-sdk-0.1.2-linux-x86_64.tar.gz` | x86_64 二进制包 |
| `revohuman-c-sdk-0.1.2-linux-aarch64.tar.gz` | aarch64 二进制包 |
| `SHA256SUMS` | 校验和 |

```bash
sha256sum -c SHA256SUMS --ignore-missing
tar -xzf revohuman-c-sdk-0.1.2-linux-$(uname -m).tar.gz
```

解压出的目录下文中记作 `<SDK>`，可以放在任意位置。包内的 `README.md` 列出了全部文件，`SHA256SUMS` 可以逐文件校验，
`BUILDINFO.txt` 记录了构建来源。

## 用法

**CMake 工程**

```cmake
find_package(revohuman 0.1 CONFIG REQUIRED)
target_link_libraries(app PRIVATE revohuman::revohuman)          # 动态库
# 或 target_link_libraries(app PRIVATE revohuman::revohuman_static)  # 静态库
```

配置时传 `-DCMAKE_PREFIX_PATH=<SDK>`。0.x 版本按次版本号兼容：`find_package(revohuman 0.1)` 只接受 0.1.y。
纯 C 工程用静态库时不用自己加 `-lstdc++`，包配置会自动补上。

**不用 CMake**

```bash
cc  app.c   -I<SDK>/include -L<SDK>/lib -lrevohuman
c++ app.cpp -I<SDK>/include -L<SDK>/lib -lrevohuman
```

**运行时找动态库**

链接动态库的程序，运行前执行：

```bash
export LD_LIBRARY_PATH=<SDK>/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

或者把 `librevohuman.so*` 拷到程序旁边，链接时加 `-Wl,-rpath,'$ORIGIN'`。链接静态库的程序不需要这一步。

## 先跑一下示例

```bash
cmake -S example -B /tmp/rh_example -DCMAKE_PREFIX_PATH=<SDK>
cmake --build /tmp/rh_example
LD_LIBRARY_PATH=<SDK>/lib /tmp/rh_example/rh_info
/tmp/rh_example/rh_info_static
```

两个程序都会打印版本号、手部模型指纹，以及全零关节角下关节表最后一个 link 的位置，退出码为 0。
二进制包里的 `share/revohuman/example/` 是同一份示例。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
