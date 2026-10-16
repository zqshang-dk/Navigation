import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# =========================================================
# 1. 文件路径
# =========================================================
file_path = r"E:\STUDY\Sophomore2\卫星算法程序设计\成果设计报告\result.txt"

save_path = r"E:\STUDY\Sophomore2\卫星算法程序设计\成果设计报告\ENU_SigmaP_error.png"


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
# 3. 提取时间，并截取首个历元开始的24小时
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
# 4. 提取 EAST、NORTH、UP、SigmaP
# =========================================================
east = df_24h["EAST/m"]
north = df_24h["NORTH/m"]
up = df_24h["UP/m"]
sigmap = df_24h["SigmaP"]


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

data_list = [east, north, up, sigmap]
label_list = ["E", "N", "U", "SigmaP"]
unit_list = ["m", "m", "m", "m"]

for i, (ax, data, label, unit) in enumerate(zip(axes, data_list, label_list, unit_list)):

    # ax.plot(
    #     time_hour,
    #     data,
    #     color="#006400",
    #     linewidth=2.5
    # )
    # SigmaP 单独设置为红色，其余 E/N/U 保持深绿色
    line_color = "red" if label == "SigmaP" else "#006400"

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
        linewidth=1.0,
        alpha=0.8
    )

    ax.tick_params(
        axis="both",
        labelsize=18,
        direction="in"
    )

    # 前三张子图固定为 -6 到 6
    if label in ["E", "N", "U"]:
        ax.set_ylim(-6, 6)

    # SigmaP 单独自动设置纵轴范围
    if label == "SigmaP":
        ymin = data.min()
        ymax = data.max()
        margin = max((ymax - ymin) * 0.15, 0.1)
        ax.set_ylim(ymin - margin, ymax + margin)


# =========================================================
# 7. 横坐标设置
# =========================================================
axes[-1].set_xlabel("Time (h)", fontsize=18)

axes[-1].set_xlim(0, 24)
axes[-1].set_xticks(np.arange(0, 25, 4))


# =========================================================
# 8. 调整子图间距
# =========================================================
plt.subplots_adjust(
    left=0.06,
    right=0.97,
    top=0.97,
    bottom=0.10,
    hspace=0.08
)


# =========================================================
# 9. 保存和显示
# =========================================================
plt.savefig(save_path, dpi=600, bbox_inches="tight")
plt.show()