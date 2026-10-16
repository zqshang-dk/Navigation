import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# =========================================================
# 1. 文件路径
# =========================================================
file_path = r"E:\STUDY\Sophomore2\卫星算法程序设计\成果设计报告\result.txt"

save_path = r"E:\STUDY\Sophomore2\卫星算法程序设计\成果设计报告\BS_GS_n_onefigure.png"


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
# 4. 提取 BS、GS、n 三列
# =========================================================
bs = df_24h["BS"]
gs = df_24h["GS"]
n = df_24h["n"]


# =========================================================
# 5. 全局绘图设置
# =========================================================
plt.rcParams["font.family"] = "Arial"
plt.rcParams["axes.unicode_minus"] = False


# =========================================================
# 6. 绘图：三条线画在同一个图中
# =========================================================
fig, ax = plt.subplots(
    figsize=(15, 5)
)

# ax.plot(
#     time_hour,
#     bs,
#     color="#006400",     # 深绿色
#     linewidth=1.8,
#     label="BS"
# )
#
# ax.plot(
#     time_hour,
#     gs,
#     color="#003366",     # 深蓝色
#     linewidth=1.8,
#     label="GS"
# )
#
# ax.plot(
#     time_hour,
#     n,
#     color="black",       # 黑色
#     linewidth=1.8,
#     label="n"
# )
ax.plot(
    time_hour,
    bs,
    color="#2FAC66",     # 浅一点的绿色
    linewidth=1.8,
    label="BDSNum"
)

ax.plot(
    time_hour,
    gs,
    color="#1F77B4",     # 浅一点的蓝色
    linewidth=1.8,
    label="GPSNum"
)

ax.plot(
    time_hour,
    n,
    color="red",         # 红色
    linewidth=1.8,
    linestyle="-",      # 虚线
    label="TotalNum"
)


# =========================================================
# 7. 坐标轴设置
# =========================================================
ax.set_xlabel("Time (h)", fontsize=18)
ax.set_ylabel("Number of Satellites", fontsize=18, labelpad=6)

ax.set_xlim(0, 24)
ax.set_xticks(np.arange(0, 25, 4))

# 纵轴范围自动扩展
ymin = min(bs.min(), gs.min(), n.min())
ymax = max(bs.max(), gs.max(), n.max())
margin = max((ymax - ymin) * 0.15, 1)

ax.set_ylim(ymin - margin, ymax + margin)

ax.grid(
    True,
    linestyle="--",
    linewidth=0.5,
    alpha=0.6
)

ax.tick_params(
    axis="both",
    labelsize=18,
    direction="in"
)


# =========================================================
# 8. 图例设置
# =========================================================
ax.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, 1.18),
    ncol=3,
    fontsize=18,
    frameon=False
)

# =========================================================
# 9. 调整布局
# =========================================================
plt.subplots_adjust(
    left=0.07,
    right=0.98,
    top=0.90,
    bottom=0.16
)


# =========================================================
# 10. 保存和显示
# =========================================================
plt.savefig(save_path, dpi=600, bbox_inches="tight")
plt.show()