import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# =========================================================
# 1. 文件路径
# =========================================================
file_path = r"E:\STUDY\Sophomore2\卫星算法程序设计\成果设计报告\result.txt"

save_path = r"E:\STUDY\Sophomore2\卫星算法程序设计\成果设计报告\VXYZ_SigmaV_error.png"


# =========================================================
# 2. 读取数据
# =========================================================
df = pd.read_csv(
    file_path,
    sep=r"\s+",
    engine="python"
)

# 去除列名可能存在的空格
df.columns = df.columns.str.strip()


# =========================================================
# 3. 根据 Week 和 SOW 构造连续时间，并截取首个历元开始的24小时
# =========================================================

# 将 GPS Week + SOW 转为连续时间，单位：秒
gps_time_sec = df["Week"] * 7 * 24 * 3600 + df["SOW"]

# 第一个历元作为起始时间
start_time = gps_time_sec.iloc[0]

# 结束时间：起始时间后 24 小时
end_time = start_time + 24 * 3600

# 截取从第一个历元开始的 24 小时数据
mask = (gps_time_sec >= start_time) & (gps_time_sec < end_time)
df_24h = df.loc[mask].copy()

# 重新计算横坐标，单位为小时，从 0 开始
gps_time_sec_24h = df_24h["Week"] * 7 * 24 * 3600 + df_24h["SOW"]
time_hour = (gps_time_sec_24h - start_time) / 3600.0


# =========================================================
# 4. 提取 VX、VY、VZ、SigmaV
# =========================================================
vx = df_24h["VX/m"]
vy = df_24h["VY/m"]
vz = df_24h["VZ/m"]
sigmav = df_24h["SigmaV"]


# =========================================================
# 5. 全局绘图设置
# =========================================================
plt.rcParams["font.family"] = "Arial"
plt.rcParams["axes.unicode_minus"] = False


# =========================================================
# 6. 绘图
# =========================================================
fig, axes = plt.subplots(
    4, 1,
    figsize=(15, 10),
    sharex=True
)

data_list = [vx, vy, vz, sigmav]
label_list = ["VX", "VY", "VZ", "SigmaV"]
unit_list = ["m/s", "m/s", "m/s", "m/s"]

for ax, data, label, unit in zip(axes, data_list, label_list, unit_list):

    # SigmaV 单独设为红色，其余保持深蓝色
    line_color = "red" if label == "SigmaV" else "#003366"

    ax.plot(
        time_hour,
        data,
        color=line_color,
        linewidth=2.5
    )

    ax.set_ylabel(
        f"{label} ({unit})",
        fontsize=18,
        labelpad=4
    )

    ax.grid(
        True,
        linestyle="--",
        linewidth=1,
        alpha=0.8
    )

    ax.tick_params(
        axis="both",
        labelsize=18,
        direction="in"
    )

    # VX、VY、VZ 固定纵轴范围
    if label in ["VX", "VY", "VZ"]:
        ax.set_ylim(-0.15, 0.15)

    # SigmaV 单独自动设置纵轴范围
    if label == "SigmaV":
        ymin = data.min()
        ymax = data.max()
        margin = max((ymax - ymin) * 0.15, 0.01)
        ax.set_ylim(ymin - margin, ymax + margin)


# =========================================================
# 7. 横坐标设置
# =========================================================
axes[-1].set_xlabel("Time (h)", fontsize=18)

axes[-1].set_xlim(0, 24)
axes[-1].set_xticks(np.arange(0, 25, 4))


# =========================================================
# 8. 调整子图布局
# =========================================================
plt.subplots_adjust(
    left=0.07,
    right=0.98,
    top=0.97,
    bottom=0.10,
    hspace=0.08
)


# =========================================================
# 9. 保存和显示
# =========================================================
plt.savefig(save_path, dpi=600, bbox_inches="tight")
plt.show()