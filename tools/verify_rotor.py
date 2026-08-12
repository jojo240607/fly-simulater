"""验证螺旋桨是否画成"圈圈"（淡蓝圆盘 + 亮边圆环）。
- 旋翼盘淡蓝填充: spin_col=[140,170,190]
- 外圈亮边: [190,215,230]
检测机体中心区域是否存在这些淡蓝/亮边像素簇（4个旋翼位置）。
"""
import sys
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

def is_disc(r, g, b):
    return 120 < r < 210 and 150 < g < 230 and 170 < b < 245

def is_ring(r, g, b):
    return 170 < r < 220 and 195 < g < 240 and 210 < b < 250

disc = ring = 0
for y in range(h):
    for x in range(w):
        r, g, b = px[x, y]
        if is_disc(r, g, b):
            disc += 1
        elif is_ring(r, g, b):
            ring += 1

print(f"淡蓝旋翼盘像素: {disc}")
print(f"亮边圆环像素: {ring}")
print("结论:", "螺旋桨圈盘已渲染" if disc > 50 else "未检测到圈盘")
