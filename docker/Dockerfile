# RevoHuman SDK runtime image: Ubuntu 22.04 + ROS 2 Humble + RevoHuman SDK (with the bundled pysurvive)
# + ROS 2 packages + visualization environment.
# The install steps correspond item by item to the user manual *01 Installation and System Requirements*,
# and the directory layout in the container matches the manual:
#   ~/revohuman-release/revohuman-<version>/   source bundle (ros2/, simulation/, examples/ …)
#   ~/ros2_ws/                                 ROS 2 workspace (already built with colcon build)
#
# Normally built in one step with install.sh in this directory (which also checks the host, verifies
# the release files, and checks the image after building).
# For a manual build, the build context is the directory holding the release files (two wheels, the
# source bundle, SHA256SUMS), e.g.:
#   cd ~/revohuman-release
#   tar xzf revohuman-0.1.4-bundle.tar.gz
#   docker build -f revohuman-0.1.4/docker/Dockerfile --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) -t revohuman:0.1.4 .
# Run with run.sh in this directory. Full details: user manual *08 Running in Containers*.

ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

# Package sources. All official by default; on networks in mainland China they can be switched to
# mirrors, e.g.:
#   --build-arg UBUNTU_MIRROR=http://mirrors.aliyun.com
#   --build-arg ROS_APT_MIRROR=https://mirrors.aliyun.com/ros2/ubuntu
#   --build-arg ROS_GPG_URL=https://mirrors.aliyun.com/rosdistro/ros.key
#   --build-arg PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/
# UBUNTU_MIRROR is the mirror site root URL (containing ubuntu/ and ubuntu-ports/); empty means no
# replacement.
# There are no CA certificates in the container yet at this point, so UBUNTU_MIRROR must be an http:// URL.
ARG UBUNTU_MIRROR=
ARG ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu
ARG ROS_GPG_URL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1. System software: ROS 2 Humble, pip/venv, Bluetooth, OpenGL/EGL, the X11 libraries used by
#    visualization windows (glfw),
#    and nano for editing the base station allowlist in the container. Failed downloads are retried
#    automatically
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
 && if [[ -n "${UBUNTU_MIRROR}" ]]; then \
        sed -i -e "s#http://archive.ubuntu.com/ubuntu#${UBUNTU_MIRROR%/}/ubuntu#g" \
               -e "s#http://security.ubuntu.com/ubuntu#${UBUNTU_MIRROR%/}/ubuntu#g" \
               -e "s#http://ports.ubuntu.com/ubuntu-ports#${UBUNTU_MIRROR%/}/ubuntu-ports#g" /etc/apt/sources.list; \
    fi \
 && grep -qE '^deb .* universe' /etc/apt/sources.list \
 && apt-get update \
 && apt-get install -y ca-certificates curl gnupg lsb-release tzdata \
 && curl -fsSL --retry 5 --retry-delay 3 --retry-all-errors "${ROS_GPG_URL}" -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_MIRROR} $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y ros-humble-ros-base ros-dev-tools \
        python3-pip python3-venv bluez libgl1 libegl1 nano \
        libxrandr2 libxinerama1 libxcursor1 libxi6 libxkbcommon0 \
 && rm -rf /var/lib/apt/lists/*

# 2. Regular user (access to serial ports, GPU and USB devices is added by run.sh using the host's
# group IDs).
# USER_UID/USER_GID must match the host user running the container: the ownership of mounted directories
# and the machine-wide Tracker lock both go by numeric ID.
# ~/.hushlogin turns off the sudo hint from /etc/bash.bashrc: it runs groups, and the host groups that
# run.sh brings in with --group-add
# (e.g. the GPU's render group) have no name in the image, so every shell would print
# groups: cannot find name for group ID …
ARG USERNAME=revohuman
ARG USER_UID=1000
ARG USER_GID=1000
RUN (getent group "${USER_GID}" >/dev/null || groupadd -g "${USER_GID}" "${USERNAME}") \
 && useradd -l -m -u "${USER_UID}" -g "${USER_GID}" -G dialout,video,plugdev -s /bin/bash "${USERNAME}" \
 && printf '%s\n' '#!/bin/bash' \
        '# 加载 ROS 2 与 RevoHuman 工作空间的环境后执行命令（docker run 不经过 ~/.bashrc）' \
        'source /opt/ros/humble/setup.bash' \
        'source "${HOME}/ros2_ws/install/setup.bash"' \
        'exec "$@"' > /usr/local/bin/revohuman-entrypoint \
 && chmod 0755 /usr/local/bin/revohuman-entrypoint \
 && install -d -o "${USER_UID}" -g "${USER_GID}" "/home/${USERNAME}/revohuman-release" "/home/${USERNAME}/data" \
 && install -m 0644 -o "${USER_UID}" -g "${USER_GID}" /dev/null "/home/${USERNAME}/.hushlogin"

USER ${USERNAME}
ENV HOME=/home/${USERNAME} \
    PATH=/home/${USERNAME}/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
WORKDIR /home/${USERNAME}/revohuman-release

# 3. Release files. Only these file types are taken; each is verified against SHA256SUMS
COPY --chown=${USER_UID}:${USER_GID} SHA256SUMS *.whl *-bundle.tar.gz ./

# 4. Pick and install the wheels for this machine's architecture; unpack the source bundle; build the
# ROS 2 workspace; set up the visualization environment.
# The two steps that download dependencies are retried automatically on network timeouts (up to 3 times).
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
 && "${HOME}/revohuman-release/${rel}/simulation/run_viewer.sh" --demo P1 --headless --seconds 2 --snapshot /tmp/revohuman-viewer-warmup.png \
 && rm -rf /tmp/revohuman-viewer-warmup.png "${HOME}/.cache/pip" \
 && mkdir -p "${HOME}/.config/libsurvive" "${HOME}/.config/revohuman" "${HOME}/.local/state/revohuman" \
 && printf '%s\n' '' '# RevoHuman' 'source /opt/ros/humble/setup.bash' 'source ~/ros2_ws/install/setup.bash' >> "${HOME}/.bashrc"

# 5. Self-check (same as the manual *01 Installation and System Requirements*, section 10).
# If any item fails, the build fails
RUN source /opt/ros/humble/setup.bash && source "${HOME}/ros2_ws/install/setup.bash" \
 && python3 -c "import revohuman, pysurvive, mcap, bleak, mujoco; print('revohuman', revohuman.__version__)" \
 && python3 -c "import pysurvive; print('libsurvive', pysurvive.survive_build_tag().decode())" \
 && python3 -m pip show pysurvive | grep '^Version' \
 && python3 -c "import numpy, sys; print('numpy', numpy.__version__); sys.exit(0 if numpy.__version__.startswith('1.') else 1)" \
 && python3 -m revohuman --help >/dev/null \
 && command -v revohuman-live revohuman-lighthouses >/dev/null \
 && ros2 pkg list | grep revohuman \
 && ros2 interface show revohuman_msgs/srv/Calibrate >/dev/null

# run.sh uses these labels to check the host user and determine mount locations
LABEL org.revohuman.uid="${USER_UID}" org.revohuman.gid="${USER_GID}" org.revohuman.home="/home/${USERNAME}"
WORKDIR /home/${USERNAME}/data
ENTRYPOINT ["/usr/local/bin/revohuman-entrypoint"]
CMD ["bash"]
