"""诊断：找出画面中接近外圈亮边色[200,220,235]的像素，打印其实际颜色分布。"""
import sys
from collections import Counter
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

cnt = Counter()
for y in range(h):
    for x in range(w):
        r, g, b = px[x, y]
        # 近亮边色：r>170,g>190,b>210 且偏亮
        if r > 170 and g > 190 and b > 210:
            cnt[(r // 4 * 4, g // 4 * 4, b // 4 * 4)] += 1

print("近亮边色像素的 Top 颜色（量化到4）:")
for col, n in cnt.most_common(10):
    print(f"  RGB{col}: {n}")
if not cnt:
    print("  无任何近亮边色像素")
