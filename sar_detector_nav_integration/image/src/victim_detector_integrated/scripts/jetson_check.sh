#!/usr/bin/env bash
# jetson_check.sh - 机器人上的只读体检脚本 (Team Lumen Fleet / SAR)
#
# 回答一个问题：这台 Jetson 上到底能不能跑我们的检测节点，缺什么。
#
# 安全性：
#   * 只读。不装任何东西、不改任何配置、不需要 sudo。
#   * 唯一的写操作是在 $TMPDIR 下建一个临时目录，退出时自己删掉。
#   * 任何一项失败都不会中断，继续跑完所有检查。
#
# 用法：
#   scp jetson_check.sh <user>@<robot>:~/
#   ssh <user>@<robot> 'bash ~/jetson_check.sh 2>&1 | tee jetson_report.txt'
#   然后把 jetson_report.txt 发回来。

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

PY="${PY:-python3}"

# 只在真终端里上色。输出重定向到文件时不带转义码，
# 这样 tee 出来的报告可以直接粘贴。
if [ -t 1 ]; then B=$'\033[1m'; R=$'\033[0m'; else B=""; R=""; fi

sect() { printf '\n%s=== %s ===%s\n' "$B" "$1" "$R"; }
kv()   { printf '  %-26s %s\n' "$1" "${2:-（无输出）}"; }
have() { command -v "$1" >/dev/null 2>&1; }

printf '%sJetson 体检报告%s  %s\n' "$B" "$R" "$(date -Is 2>/dev/null || date)"
kv "主机" "$(hostname 2>/dev/null)"
kv "用户" "$(id -un 2>/dev/null) (uid $(id -u 2>/dev/null))"
kv "sudo 可用" "$(sudo -n true 2>/dev/null && echo yes || echo 'no (预期如此)')"

# ssh 非交互执行时 Ubuntu 的 ~/.bashrc 在开头就 return，ROS 永远不会被 source，
# 于是后面会假报「rclpy 缺失」「没有相机话题」。所以在任何检查之前自己 source。
# 这是只读操作，只改本进程的环境变量。
if [ -z "${ROS_DISTRO:-}" ]; then
    for s in /opt/ros/*/setup.bash; do
        [ -r "$s" ] || continue
        # shellcheck disable=SC1090
        . "$s" >/dev/null 2>&1 && kv "自动 source" "$s"
        break
    done
    for s in "$HOME"/*_ws/install/setup.bash; do
        [ -r "$s" ] || continue
        # shellcheck disable=SC1090
        . "$s" >/dev/null 2>&1 && kv "自动 source" "$s"
        break
    done
fi

# ---------------------------------------------------------------- 平台
sect "1. 平台"
kv "架构" "$(uname -m)"
kv "内核" "$(uname -r)"
[ -r /etc/os-release ] && kv "发行版" "$(. /etc/os-release; echo "$PRETTY_NAME")"

if [ -r /sys/firmware/devicetree/base/model ]; then
    kv "板型" "$(tr -d '\0' < /sys/firmware/devicetree/base/model)"
fi
if [ -r /etc/nv_tegra_release ]; then
    kv "L4T" "$(head -1 /etc/nv_tegra_release)"
else
    kv "L4T" "/etc/nv_tegra_release 不存在 -- 这可能不是 Jetson"
fi
# JetPack 版本（apt-cache 不需要 sudo）
if have apt-cache; then
    jp=$(apt-cache policy nvidia-jetpack 2>/dev/null | awk '/Installed:/{print $2}')
    kv "JetPack" "${jp:-未通过 apt 安装 / 查不到}"
fi

# ---------------------------------------------------------------- CUDA
sect "2. CUDA / 驱动 / TensorRT"
NVCC=""
for c in nvcc /usr/local/cuda/bin/nvcc /usr/local/cuda-13.2/bin/nvcc; do
    if have "$c" || [ -x "$c" ]; then NVCC="$c"; break; fi
done
if [ -n "$NVCC" ]; then
    rel=$("$NVCC" --version 2>/dev/null | awk '/release/{print $5$6}' | tr -d ',')
    kv "nvcc" "$NVCC"
    kv "CUDA release" "${rel:-?}"
    # 这一行决定 pip 该用哪个 index
    short=$(printf '%s' "$rel" | tr -d 'V' | awk -F. '{printf "cu%d%d", $1, $2}')
    kv "-> 应使用的 pip index" "https://download.pytorch.org/whl/${short}"
else
    kv "nvcc" "找不到。CUDA 可能没装，或不在 PATH（试试 /usr/local/cuda/bin）"
fi
[ -e /usr/local/cuda ] && kv "cuda 软链接" "$(readlink -f /usr/local/cuda)"

if have nvidia-smi; then
    kv "nvidia-smi" "存在"
    nvidia-smi --query-gpu=name,driver_version,compute_cap --format=csv,noheader 2>/dev/null \
        | sed 's/^/    /' || echo "    (nvidia-smi 查询失败，Jetson 上常见，不影响)"
else
    kv "nvidia-smi" "不存在（部分 JetPack 版本如此，不一定是问题）"
fi

# TensorRT：我们导出 engine 时必须和运行时版本一致
trt=$(dpkg-query -W -f='${Version}' 'libnvinfer*' 2>/dev/null | head -1)
kv "TensorRT (dpkg)" "${trt:-查不到}"
$PY - <<'EOF' 2>/dev/null || echo "  TensorRT python 绑定: 不可用"
try:
    import tensorrt
    print("  %-26s %s" % ("TensorRT (python)", tensorrt.__version__))
except Exception as e:
    print("  %-26s 不可用 (%s)" % ("TensorRT (python)", type(e).__name__))
EOF

# ---------------------------------------------------------------- 功耗
sect "3. 功耗 / 时钟（决定实际帧率）"
if have nvpmodel; then
    m=$(nvpmodel -q 2>/dev/null | tr '\n' ' ')
    kv "nvpmodel" "${m:-查询需要 sudo，跳过}"
else
    kv "nvpmodel" "不存在"
fi
if have tegrastats; then
    s=$(timeout 3 tegrastats --interval 1000 2>/dev/null | head -1)
    kv "tegrastats" "${s:-需要 sudo 或超时，跳过}"
else
    kv "tegrastats" "不存在"
fi
[ -r /sys/devices/system/cpu/online ] && kv "在线 CPU" "$(cat /sys/devices/system/cpu/online)"

# ---------------------------------------------------------------- Python
sect "4. Python 环境"
kv "python3" "$(command -v $PY || echo 缺失)"
kv "版本" "$($PY -VV 2>&1 | tr '\n' ' ')"
kv "pip" "$($PY -m pip --version 2>&1 | head -1)"

# ---------------------------------------------------------------- torch
sect "5. PyTorch（最关键的一项）"
# 注意：import torch 成功 + cuda.is_available() 为 True 都证明不了什么。
# 架构不对的 wheel 要到真正启动 kernel 时才会炸。所以这里必须做一次真实的
# GPU 矩阵乘法，并且把结果搬回 CPU 强制同步。
$PY - <<'EOF'
import sys


def line(k, v):
    print("  %-26s %s" % (k, v))


try:
    import torch
except Exception as e:
    line("torch", "未安装 / 导入失败")
    line("  错误", "%s: %s" % (type(e).__name__, e))
    sys.exit(0)

line("torch", torch.__version__)
line("编译时 CUDA", torch.version.cuda)
line("cuda.is_available()", torch.cuda.is_available())
# 这是最能提前发现问题的一行。Orin 是 sm_87；wheel 的架构列表里没有 sm_87
# 的话，import 和 is_available() 都会照常通过，等到真正跑推理才炸。
archs = []
try:
    archs = list(torch.cuda.get_arch_list())
    line("支持的架构", " ".join(archs) or "(空)")
except Exception:
    pass

if archs and not any(a in ("sm_87", "compute_87") for a in archs):
    line("!! 架构警告", "wheel 不含 sm_87 -- Orin 上会报 no kernel image")

if not torch.cuda.is_available():
    line("判定", "!! torch 看不到 GPU -- 会退化到 CPU，帧率不可用")
    sys.exit(0)

try:
    line("设备名", torch.cuda.get_device_name(0))
    cap = torch.cuda.get_device_capability(0)
    line("算力 (compute cap)", "sm_%d%d  (Orin 应为 sm_87)" % cap)
except Exception as e:
    line("设备查询失败", e)

# 真实 kernel 启动测试
try:
    import time
    a = torch.randn(512, 512, device="cuda")
    b = torch.randn(512, 512, device="cuda")
    c = (a @ b).sum().item()          # .item() 强制同步，错误在这里才会暴露
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    for _ in range(20):
        a @ b
    torch.cuda.synchronize()
    dt = (time.perf_counter() - t0) / 20 * 1000
    line("GPU kernel 测试", "通过 (sum=%.1f, 512x512 matmul %.2f ms)" % (c, dt))
    line("判定", "OK - torch 可用于推理")
except Exception as e:
    line("GPU kernel 测试", "!! 失败: %s: %s" % (type(e).__name__, e))
    line("判定", "!! wheel 架构不对。重装：pip install torch torchvision "
                 "--index-url https://download.pytorch.org/whl/cuXXX")
EOF

# ---------------------------------------------------------------- 其余依赖
sect "6. 其余 Python 依赖"
$PY - <<'EOF'
mods = [("torchvision", "torchvision"), ("ultralytics", "ultralytics"),
        ("numpy", "numpy"), ("cv2", "opencv"), ("rclpy", "rclpy"),
        ("cv_bridge", "cv_bridge"), ("vision_msgs.msg", "vision_msgs"),
        ("onnxruntime", "onnxruntime")]
for mod, label in mods:
    try:
        m = __import__(mod, fromlist=["__version__"])
        v = getattr(m, "__version__", "(无版本号)")
        note = ""
        if label == "numpy" and str(v).startswith("2"):
            note = "   <-- !! 必须 <2，否则 cv_bridge 导入即崩"
        print("  %-26s %s%s" % (label, v, note))
    except Exception as e:
        print("  %-26s 缺失 (%s)" % (label, type(e).__name__))
EOF

# ---------------------------------------------------------------- Docker
sect "7. Docker（团队的部署方式）"
if have docker; then
    dv=$(timeout 10 docker --version 2>&1 | head -1)
    kv "docker" "${dv:-在 PATH 里但 --version 没输出（可疑：壳脚本/权限问题）}"
    if timeout 15 docker info >/dev/null 2>&1; then
        kv "daemon 可访问" "yes"
        rt=$(docker info --format '{{json .Runtimes}}' 2>/dev/null)
        kv "runtimes" "${rt:-?}"
        case "$rt" in
            *nvidia*) kv "nvidia runtime" "有 -- 容器内可用 GPU" ;;
            *)        kv "nvidia runtime" "!! 没有，容器里会看不到 GPU" ;;
        esac
        kv "本地镜像数" "$(docker images -q 2>/dev/null | wc -l)"
    else
        kv "daemon 可访问" "no -- 当前用户可能不在 docker 组（需要管理员加）"
    fi
else
    kv "docker" "未安装"
fi

# ---------------------------------------------------------------- 相机
sect "8. 相机 / ROS"
ls /dev/video* >/dev/null 2>&1 && kv "/dev/video*" "$(ls /dev/video* | tr '\n' ' ')" \
                               || kv "/dev/video*" "没有 video 设备"
if have lsusb; then
    rs=$(lsusb 2>/dev/null | grep -i intel)
    kv "USB 上的 Intel 设备" "${rs:-没找到（D435i 没插？）}"
fi

kv "ROS_DISTRO" "${ROS_DISTRO:-未设置，且 /opt/ros 下找不到 setup.bash}"
if have ros2; then
    kv "ros2 CLI" "有"
    topics=$(timeout 8 ros2 topic list 2>/dev/null | grep -i -E 'image|camera|color|depth' | head -12)
    if [ -n "$topics" ]; then
        printf '  相机话题:\n%s\n' "$(printf '%s' "$topics" | sed 's/^/    /')"
    else
        kv "相机话题" "没发现（相机节点没跑，或 ROS_DOMAIN_ID 不同）"
    fi
    kv "ROS_DOMAIN_ID" "${ROS_DOMAIN_ID:-未设置 (=0)}"
else
    kv "ros2 CLI" "不在 PATH"
fi

# ---------------------------------------------------------------- 资源
sect "9. 资源"
kv "内存" "$(free -h 2>/dev/null | awk '/^Mem:/{print $2" 总 / "$7" 可用"}')"
kv "根分区" "$(df -h / 2>/dev/null | awk 'NR==2{print $4" 可用 / "$2" 共  ("$5" 已用)"}')"
if [ -d /var/lib/docker ]; then
    kv "/var/lib/docker 所在" "$(df -h /var/lib/docker 2>/dev/null | awk 'NR==2{print $4" 可用"}')"
fi
kv "联网" "$(timeout 5 getent hosts pypi.org >/dev/null 2>&1 && echo '可以解析 pypi.org' || echo '解析不了 pypi.org -- 离线')"

sect "完成"
echo "  把完整输出发回去。重点看第 5 节的「判定」行。"
