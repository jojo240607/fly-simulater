"""验证旋翼盘是否投影成椭圆（与机体同一水平面），而非正圆。
只在机体中心区域检测，找 4 个独立小圆盘簇，比较每盘横向/纵向尺寸。
机体水平+斜视角时旋翼盘应为横向扁椭圆（w > h）。
"""
import sys
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

# 旋翼盘淡蓝 [140,170,190]；内圈 [115,138,158]；收紧阈值
def is_disc(r, g, b):
    return 120 < r < 190 and 150 < g < 220 and 160 < b < 230

# 限定机体中心区域（旋翼在机体附近）
x0, x1 = int(w*0.25), int(w*0.75)
y0, y1 = int(h*0.25), int(h*0.75)

seen = [[False]*w for _ in range(h)]
clusters = []
for y in range(y0, y1):
    for x in range(x0, x1):
        r, g, b = px[x, y]
        if is_disc(r, g, b) and not seen[y][x]:
            stack = [(x, y)]; seen[y][x] = True; pts = []
            while stack:
                cx, cy = stack.pop(); pts.append((cx, cy))
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        nx, ny = cx+dx, cy+dy
                        if 0<=nx<w and 0<=ny<h and not seen[ny][nx]:
                            rr,gg,bb = px[nx,ny]
                            if is_disc(rr,gg,bb):
                                seen[ny][nx]=True; stack.append((nx,ny))
            if 30 < len(pts) < 30000:  # 排除超大背景簇和极小噪点
                xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
                clusters.append((len(pts), max(xs)-min(xs), max(ys)-min(ys)))

print("=== 旋翼盘形状分析（中心区域） ===")
print(f"检测到簇数: {len(clusters)}")
for i,(n,wd,ht) in enumerate(sorted(clusters, key=lambda c:-c[0])[:8]):
    ratio = wd/max(1,ht)
    kind = "横向椭圆" if ratio>1.2 else ("纵向椭圆" if ratio<0.83 else "近正圆")
    print(f"  盘{i}: 像素={n} 宽={wd} 高={ht} 宽高比={ratio:.2f} -> {kind}")
