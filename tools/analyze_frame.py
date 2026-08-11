"""聚焦分析：机身纹理（碳纤维+警示条）与旋翼盘纹理是否渲染在画面上。
在画面中心区域采样像素，检测：
- 机身警示条：黄色(225,190,60)
- 机身碳纤维：暗灰蓝(40-55,46-60,52-68)
- 螺旋桨纹理：绿色调(g>150)
"""
import sys
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

# 中心区域（机体大致在中心）
cx0, cx1 = int(w*0.30), int(w*0.70)
cy0, cy1 = int(h*0.30), int(h*0.70)

yellow = carbon = green = 0
for y in range(cy0, cy1, 1):
    for x in range(cx0, cx1, 1):
        r, g, b = px[x, y]
        if r > 190 and g > 150 and b < 110:
            yellow += 1  # 警示条黄
        elif 30 < r < 70 and 35 < g < 75 and 40 < b < 85:
            carbon += 1  # 碳纤维暗灰
        elif g > 150 and r < 140 and 80 < b < 180:
            green += 1  # 螺旋桨绿

print(f"中心区域({cx0}-{cx1},{cy0}-{cy1})")
print("机身警示条黄像素:", yellow)
print("机身碳纤维暗灰像素:", carbon)
print("螺旋桨绿色像素:", green)
print("结论:", "机身纹理可见" if carbon > 30 or yellow > 10 else "机身纹理不明显")
