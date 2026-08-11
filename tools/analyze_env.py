"""分析 canvas 截图，确认环境贴图元素是否存在：
- 天空渐变：顶部天空蓝 vs 底部暗色
- 地面十字标：红色(230,80,70) 像素
- 朝向指示器：左下角区域的红/绿/蓝三色像素
"""
import sys
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

# 1) 天空渐变：采样顶部/中部/底部若干点的颜色
def avg(y0, y1):
    rs = gs = bs = n = 0
    for y in range(y0, y1, 6):
        for x in range(0, w, 8):
            r, g, b = px[x, y]
            rs += r; gs += g; bs += b; n += 1
    return (rs // n, gs // n, bs // n)

top = avg(0, max(1, h // 6))
mid = avg(h // 3, 2 * h // 3)
bot = avg(2 * h // 3, h)
print("天空渐变(顶部/中部/底部RGB):", top, mid, bot)

# 2) 地面十字标：红色(230,80,70)像素数
red = 0
for y in range(0, h, 2):
    for x in range(0, w, 2):
        r, g, b = px[x, y]
        if r > 180 and g < 130 and b < 120:
            red += 1
print("红色像素(地面十字标):", red)

# 3) 朝向指示器：左下角区域(w*0..0.25, h*0.7..h)的红/绿/蓝像素
def count_color(x0, x1, y0, y1, pred):
    c = 0
    for y in range(y0, y1):
        for x in range(x0, x1):
            r, g, b = px[x, y]
            if pred(r, g, b):
                c += 1
    return c

lx0, lx1 = 0, int(w * 0.3)
ly0, ly1 = int(h * 0.7), h
comp_red = count_color(lx0, lx1, ly0, ly1, lambda r, g, b: r > 180 and g < 130 and b < 120)
comp_green = count_color(lx0, lx1, ly0, ly1, lambda r, g, b: g > 150 and r < 130 and b < 150)
comp_blue = count_color(lx0, lx1, ly0, ly1, lambda r, g, b: b > 180 and r < 150 and g < 170)
print("左下角朝向指示器 红/绿/蓝 像素:", comp_red, comp_green, comp_blue)
