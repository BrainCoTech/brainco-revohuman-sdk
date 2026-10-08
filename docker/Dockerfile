# RevoHuman SDK 运行镜像：Ubuntu 22.04 + ROS 2 Humble + RevoHuman SDK（含随附的 pysurvive）+ ROS 2 包 + 可视化环境。
# 安装步骤与使用手册《01-环境安装与系统要求》逐项对应，容器里的目录布局也与手册相同：
#   ~/revohuman-release/revohuman-<版本>/   发布包（ros2/、simulation/、examples/ …）
#   ~/ros2_ws/                              ROS 2 工作空间（已 colcon build）
#
# 构建上下文是存放发布文件的目录（两个 wheel、bundle、SHA256SUMS），例如：
#   cd ~/revohuman-release
#   tar xzf revohuman-0.1.0-bundle.tar.gz
#   docker build -f revohuman-0.1.0/docker/Dockerfile --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) -t revohuman:0.1.0 .
# 运行用同目录的 run.sh。完整说明见使用手册《08-容器运行》。

ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

# 软件源。默认全部用官方源；国内网络可换成镜像，例如：
#   --build-arg UBUNTU_MIRROR=http://mirrors.aliyun.com
#   --build-arg ROS_APT_MIRROR=https://mirrors.aliyun.com/ros2/ubuntu
#   --build-arg PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/
# UBUNTU_MIRROR 填镜像站根地址（其下有 ubuntu/ 与 ubuntu-ports/），留空则不替换。
# 此时容器里还没有 CA 证书，UBUNTU_MIRROR 要用 http:// 地址。
ARG UBUNTU_MIRROR=
ARG ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu
ARG ROS_GPG_URL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1. 系统软件：ROS 2 Humble、pip/venv、蓝牙、OpenGL/EGL，以及可视化窗口（glfw）用到的 X11 库
RUN if [[ -n "${UBUNTU_MIRROR}" ]]; then \
        sed -i -e "s#http://archive.ubuntu.com/ubuntu#${UBUNTU_MIRROR%/}/ubuntu#g" \
               -e "s#http://security.ubuntu.com/ubuntu#${UBUNTU_MIRROR%/}/ubuntu#g" \
               -e "s#http://ports.ubuntu.com/ubuntu-ports#${UBUNTU_MIRROR%/}/ubuntu-ports#g" /etc/apt/sources.list; \
    fi \
 && grep -qE '^deb .* universe' /etc/apt/sources.list \
 && apt-get update \
 && apt-get install -y ca-certificates curl gnupg lsb-release tzdata \
 && curl -sSL "${ROS_GPG_URL}" -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_MIRROR} $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y ros-humble-ros-base ros-dev-tools \
        python3-pip python3-venv bluez libgl1 libegl1 \
        libxrandr2 libxinerama1 libxcursor1 libxi6 libxkbcommon0 \
 && rm -rf /var/lib/apt/lists/*

# 2. 普通用户（串口、显卡、USB 设备的访问由 run.sh 按宿主的组号补上）。
# USER_UID/USER_GID 要与宿主上运行容器的用户一致：挂载目录的属主、Tracker 整机锁都按数字 ID 判断。
ARG USERNAME=revohuman
ARG USER_UID=1000
ARG USER_GID=1000
RUN (getent group "${USER_GID}" >/dev/null || groupadd -g "${USER_GID}" "${USERNAME}") \
 && useradd -m -u "${USER_UID}" -g "${USER_GID}" -G dialout,video,plugdev -s /bin/bash "${USERNAME}" \
 && printf '%s\n' '#!/bin/bash' \
        '# 加载 ROS 2 与 RevoHuman 工作空间的环境后执行命令（docker run 不经过 ~/.bashrc）' \
        'source /opt/ros/humble/setup.bash' \
        'source "${HOME}/ros2_ws/install/setup.bash"' \
        'exec "$@"' > /usr/local/bin/revohuman-entrypoint \
 && chmod 0755 /usr/local/bin/revohuman-entrypoint \
 && install -d -o "${USER_UID}" -g "${USER_GID}" "/home/${USERNAME}/revohuman-release" "/home/${USERNAME}/data"

USER ${USERNAME}
ENV HOME=/home/${USERNAME} \
    PATH=/home/${USERNAME}/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
WORKDIR /home/${USERNAME}/revohuman-release

# 3. 发布文件。只收用得到的这几类文件；逐个核对 SHA256SUMS
COPY --chown=${USER_UID}:${USER_GID} SHA256SUMS *.whl *-bundle.tar.gz ./

# 4. 按本机架构选 wheel 并安装；解开 bundle；构建 ROS 2 工作空间；建可视化环境。
# 下载依赖的两步网络超时会自动重试（最多 3 次）。
# 手部模型（URDF）即将随发布提供：bundle 里带 description/urdf/ 时才顺带跑一次演示预热，没有就跳过。
ARG PIP_INDEX_URL=
RUN [[ -n "${PIP_INDEX_URL}" ]] || unset PIP_INDEX_URL; \
    export PIP_DEFAULT_TIMEOUT=120 PIP_RETRIES=5 PIP_DISABLE_PIP_VERSION_CHECK=1; \
    retry() { local n; for n in 1 2 3; do "$@" && return 0; echo "第 ${n} 次失败，5 秒后重试：$*" >&2; sleep 5; done; return 1; }; \
    arch="$(uname -m)" \
 && shopt -s nullglob \
 && pys=(pysurvive-*-cp310-cp310-linux_"${arch}".whl) rev=(revohuman-*-cp310-cp310-linux_"${arch}".whl) bun=(revohuman-*-bundle.tar.gz) \
 && if [[ ${#pys[@]} != 1 || ${#rev[@]} != 1 || ${#bun[@]} != 1 ]]; then \
        echo "构建上下文里要各有且只有一个：pysurvive-*-${arch}.whl、revohuman-*-${arch}.whl、revohuman-*-bundle.tar.gz"; \
        echo "实际：${pys[*]:-无} | ${rev[*]:-无} | ${bun[*]:-无}"; exit 1; \
    fi \
 && for f in "${pys[0]}" "${rev[0]}" "${bun[0]}"; do \
        awk -v f="${f}" '$2 == f' SHA256SUMS | sha256sum -c - || { echo "SHA256SUMS 里没有 ${f}，或校验不符"; exit 1; }; \
    done \
 && python3 -m pip install --user --no-deps "./${pys[0]}" \
 && retry python3 -m pip install --user "./${rev[0]}[recording,camera,ble,viewer]" \
 && rel="$(tar tzf "${bun[0]}" | awk -F/ 'NR == 1 {print $1}')" \
 && tar xzf "${bun[0]}" \
 && mkdir -p "${HOME}/ros2_ws/src" \
 && ln -s "${HOME}/revohuman-release/${rel}"/ros2/* "${HOME}/ros2_ws/src/" \
 && cd "${HOME}/ros2_ws" \
 && source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install \
 && retry "${HOME}/revohuman-release/${rel}/simulation/run_viewer.sh" --help >/dev/null \
 && if [[ -d "${HOME}/revohuman-release/${rel}/description/urdf" ]]; then \
        "${HOME}/revohuman-release/${rel}/simulation/run_viewer.sh" --demo P1 --headless --seconds 2 --snapshot /tmp/revohuman-viewer-warmup.png; \
    fi \
 && rm -rf /tmp/revohuman-viewer-warmup.png "${HOME}/.cache/pip" \
 && mkdir -p "${HOME}/.config/libsurvive" "${HOME}/.config/revohuman" "${HOME}/.local/state/revohuman" \
 && printf '%s\n' '' '# RevoHuman' 'source /opt/ros/humble/setup.bash' 'source ~/ros2_ws/install/setup.bash' >> "${HOME}/.bashrc"

# 5. 自检（与手册《01-环境安装与系统要求》第 10 节相同）。任何一项不成立，构建就失败
RUN source /opt/ros/humble/setup.bash && source "${HOME}/ros2_ws/install/setup.bash" \
 && python3 -c "import revohuman, pysurvive, mcap, bleak, mujoco; print('revohuman', revohuman.__version__)" \
 && python3 -c "import pysurvive; print('libsurvive', pysurvive.survive_build_tag().decode())" \
 && python3 -m pip show pysurvive | grep '^Version' \
 && python3 -c "import numpy, sys; print('numpy', numpy.__version__); sys.exit(0 if numpy.__version__.startswith('1.') else 1)" \
 && python3 -m revohuman --help >/dev/null \
 && command -v revohuman-live revohuman-lighthouses >/dev/null \
 && ros2 pkg list | grep revohuman \
 && ros2 interface show revohuman_msgs/srv/Calibrate >/dev/null

# run.sh 按这几个标签核对宿主用户、确定挂载位置
LABEL org.revohuman.uid="${USER_UID}" org.revohuman.gid="${USER_GID}" org.revohuman.home="/home/${USERNAME}"
WORKDIR /home/${USERNAME}/data
ENTRYPOINT ["/usr/local/bin/revohuman-entrypoint"]
CMD ["bash"]
