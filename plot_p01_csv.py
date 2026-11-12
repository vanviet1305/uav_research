#!/usr/bin/env python3
"""Plot C++ P01 Case I CSV; uses the same tracked quantities as MATLAB."""
import csv
import sys
import matplotlib.pyplot as plt


# 1. Đọc CSV do chương trình p01_offline_case1 tạo ra.
# Đối số đầu tiên cho phép chọn file; mặc định dùng tên bên dưới.
csv_path = sys.argv[1] if len(sys.argv) > 1 else "p01_case1_cpp.csv"
with open(csv_path, encoding="utf-8", newline="") as stream:
    rows = list(csv.DictReader(stream))

if not rows:
    raise SystemExit("CSV is empty")

def values(key):
    # Chuyển một cột dữ liệu CSV từ chuỗi sang số để vẽ theo thời gian.
    return [float(row[key]) for row in rows]

# 2. So sánh vị trí thực/mong muốn, sai số và đầu vào điều khiển.
# Biểu đồ cuối dùng chung trục tung cho lực (N) và moment (N.m).
t = values("t")
fig, axes = plt.subplots(2, 2, figsize=(11, 7))
for idx in (1, 2, 3):
    axes[0, 0].plot(t, values(f"x{idx}"), label=f"x{idx}")
    axes[0, 0].plot(t, values(f"xd{idx}"), "--", label=f"xd{idx}")
axes[0, 0].set_title("Actual vs desired position")
axes[0, 0].legend(ncol=3, fontsize=8)
axes[0, 1].plot(t, values("ex_norm"))
axes[0, 1].set_title("Position error norm (m)")
axes[1, 0].plot(t, values("Psi"))
axes[1, 0].set_title("Attitude error function Psi")
axes[1, 1].plot(t, values("f"), label="f (N)")
axes[1, 1].plot(t, values("M1"), label="M1 (N.m)")
axes[1, 1].plot(t, values("M2"), label="M2 (N.m)")
axes[1, 1].plot(t, values("M3"), label="M3 (N.m)")
axes[1, 1].set_title("Control input (note different units)")
axes[1, 1].legend()
# 3. Hoàn thiện nhãn, lưới và hiển thị cửa sổ đồ thị.
for ax in axes.flat:
    ax.set_xlabel("Time (s)")
    ax.grid(True)
fig.tight_layout()
plt.show()
