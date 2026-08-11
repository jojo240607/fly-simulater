"""验证：机体三条坐标轴线是否已去掉；并定位远端残留绿色像素的来源。"""
import sys
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

def is_pure(t): return max(t) - min(t) > 80 and max(t) > 120

# 机体中心
cx0, cx1, cy0, cy1 = int(w*0.3), int(w*0.7), int(h*0.3), int(h*0.7)
samples = []
for y in range(h):
    for x in range(w):
        r, g, b = px[x, y]
        if not is_pure((r, g, b)):
            continue
        zone = "center" if (cx0 < x < cx1 and cy0 < y < cy1) else ("compass" if (x < cx0 and y > cy1) else "far")
        if zone == "far" and g > 150 and r < 100 and b < 110:
            samples.append((x, y, r, g, b))

print(f"far 绿色像素数: {len(samples)}")
# 聚类显示前几个位置
if samples:
    print("前5个位置:", samples[:5])
    # 统计位置范围
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    print(f"x范围 {min(xs)}-{max(xs)}, y范围 {min(ys)}-{max(ys)}")
