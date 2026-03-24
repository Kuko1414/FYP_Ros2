#!/bin/bash

set -e  # 遇到错误立即退出

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

info "检查操作系统版本..."
if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
    warn "当前系统可能不是 Ubuntu 22.04，脚本仍会尝试继续，但可能出现兼容性问题。"
fi

# ======================== 1. 安装 ROS 2 Humble ========================
install_ros2_humble() {
    if [ -f /opt/ros/humble/setup.bash ]; then
        info "ROS 2 Humble 已安装，跳过。"
        return
    fi

    info "开始安装 ROS 2 Humble..."

    # 确保 locale 正确
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # 添加 ROS 2 apt 源
    sudo apt install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-desktop

    info "ROS 2 Humble 安装完成。"
}

# ======================== 2. 安装构建工具 ========================
install_build_tools() {
    info "安装 ROS 2 构建工具 (colcon, rosdep, ament-cmake 等)..."
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep2 \
        python3-pip \
        python3-setuptools \
        build-essential \
        cmake
    info "构建工具安装完成。"
}

# ======================== 3. 安装 ROS 2 包依赖 ========================
install_ros2_packages() {
    info "安装本工作区所需的 ROS 2 包..."

    # source ROS 2 环境
    source /opt/ros/humble/setup.bash

    sudo apt install -y \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-std-msgs \
        ros-humble-sensor-msgs \
        ros-humble-vrpn-mocap \
        ros-humble-ament-cmake \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime

    info "ROS 2 包依赖安装完成。"
}

# ======================== 4. 初始化 rosdep ========================
init_rosdep() {
    info "初始化 rosdep..."
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init 2>/dev/null || true
    fi
    rosdep update --rosdistro=humble || true
    info "rosdep 初始化完成。"
}

# ======================== 5. 使用 rosdep 安装工作区依赖 ========================
install_workspace_deps() {
    info "使用 rosdep 安装工作区中所有包的依赖..."

    source /opt/ros/humble/setup.bash

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "$SCRIPT_DIR"

    rosdep install --from-paths src --ignore-src -r -y || true

    info "工作区依赖安装完成。"
}

# ======================== 6. 安装其他额外依赖 (pip / apt) ========================
install_extra_deps() {
    info "安装其他额外依赖..."

    # -------- apt 包 (每行一个，按需添加) --------
    APT_PACKAGES=(
        # 示例: sudo apt install -y <package_name>
        # 在下方添加你需要的 apt 包，例如:
        # "python3-opencv"
        # "ffmpeg"

    )

    if [ ${#APT_PACKAGES[@]} -gt 0 ]; then
        info "安装 apt 额外包: ${APT_PACKAGES[*]}"
        sudo apt install -y "${APT_PACKAGES[@]}"
    else
        info "没有额外的 apt 包需要安装。"
    fi

    # -------- pip 包 (每行一个，按需添加) --------
    PIP_PACKAGES=(
        # 每行只写包名（可带版本号），不需要写 pip3 install 命令
        # 例如: "numpy" 或 "numpy>=1.21"
        "google-genai"
        "numpy"
        "Pillow"
    )

    if [ ${#PIP_PACKAGES[@]} -gt 0 ]; then
        info "安装 pip 额外包: ${PIP_PACKAGES[*]}"
        pip3 install -U "${PIP_PACKAGES[@]}"
    else
        info "没有额外的 pip 包需要安装。"
    fi

    info "额外依赖安装完成。"
}

# ======================== 7. 配置 shell 环境 ========================
setup_shell_env() {
    info "配置 shell 环境..."

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    BASHRC="$HOME/.bashrc"

    # 添加 ROS 2 Humble source
    if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC" 2>/dev/null; then
        echo "" >> "$BASHRC"
        echo "# ROS 2 Humble" >> "$BASHRC"
        echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
        info "已将 'source /opt/ros/humble/setup.bash' 添加到 ~/.bashrc"
    else
        info "~/.bashrc 中已包含 ROS 2 Humble source，跳过。"
    fi

    # 添加工作区 install source (如果 install 目录存在)
    INSTALL_SETUP="$SCRIPT_DIR/install/setup.bash"
    if [ -f "$INSTALL_SETUP" ]; then
        if ! grep -q "$INSTALL_SETUP" "$BASHRC" 2>/dev/null; then
            echo "source $INSTALL_SETUP" >> "$BASHRC"
            info "已将工作区 install/setup.bash 添加到 ~/.bashrc"
        fi
    fi

    info "shell 环境配置完成。"
}

# ======================== 8. 编译工作区 ========================
build_workspace() {
    info "编译工作区..."

    source /opt/ros/humble/setup.bash

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "$SCRIPT_DIR"

    colcon build --symlink-install

    info "工作区编译完成。"
}

# ======================== 主流程 ========================
main() {
    echo ""
    echo "============================================================"
    echo "  humble_ws 依赖安装脚本"
    echo "  系统要求: Ubuntu 22.04 + ROS 2 Humble"
    echo "============================================================"
    echo ""

    install_ros2_humble
    install_build_tools
    install_ros2_packages
    init_rosdep
    install_workspace_deps
    install_extra_deps
    setup_shell_env

    echo ""
    info "所有依赖安装完成！"
    echo ""

    # 询问是否编译
    read -p "是否立即编译工作区？(y/N): " BUILD_CHOICE
    if [[ "$BUILD_CHOICE" =~ ^[Yy]$ ]]; then
        build_workspace
        echo ""
        info "编译完成！请运行以下命令刷新环境："
        echo "  source ~/.bashrc"
    else
        info "跳过编译。你可以稍后手动运行："
        echo "  cd $(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        echo "  source /opt/ros/humble/setup.bash"
        echo "  colcon build --symlink-install"
        echo "  source install/setup.bash"
    fi

    echo ""
    info "安装脚本执行完毕。"
}

main "$@"
