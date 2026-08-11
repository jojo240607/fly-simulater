"""分析 canvas 截图：检测旋翼盘（绿色圆点）的几何排列，判断机体是否水平。
旋翼盘正常色 ~(70,200,120)。若机体水平（X 构型），4 盘在画面对称分布；
若侧躺，旋翼盘会沿某一方向堆叠/不对。
"""
import sys
from PIL import Image

path = sys.argv[1] if len(sys.argv) > 1 else "web_check_canvas.png"
img = Image.open(path).convert("RGB")
w, h = img.size
px = img.load()

# 收集接近旋翼绿色(70,200,120)且够大的连通簇的质心
clusters = []
seen = [[False] * w for _ in range(h)]
for y in range(h):
    for x in range(w):
        r, g, b = px[x, y]
        # 绿色旋翼盘
        if g > 150 and r < 130 and b < 160 and not seen[y][x]:
            # BFS 找连通域
            stack = [(x, y)]
            seen[y][x] = True
            pts = []
            while stack:
                cx, cy = stack.pop()
                pts.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < w and 0 <= ny < h and not seen[ny][nx]:
                            rr, gg, bb = px[nx, ny]
                            if gg > 150 and rr < 130 and bb < 160:
                                seen[ny][nx] = True
                                stack.append((nx, ny))
            if len(pts) > 30:  # 足够大才算旋翼盘
                ax = sum(p[0] for p in pts) / len(pts)
                ay = sum(p[1] for p in pts) / len(pts)
                clusters.append((ax, ay, len(pts)))

print("=== 旋翼盘簇分析 ===")
print("检测到绿色旋翼盘簇数:", len(clusters))
for i, (cx, cy, n) in enumerate(clusters):
    print(f"  盘{i}: 中心=({cx:.0f},{cy:.0f}) 像素={n}")

if len(clusters) >= 2:
    xs = [c[0] for c in clusters]
    ys = [c[1] for c in clusters]
    print("X 范围:", round(min(xs)), "-", round(max(xs)))
    print("Y 范围:", round(min(ys)), "-", round(max(ys)))
    # 若机体水平，旋翼盘应大致成 X 型对称：两个盘在左/上，两个在右/下。
    # 粗略判据：所有盘 Y 坐标跨度应显著（机体非被压缩成一点）。
    span = max(ys) - min(ys)
    print("旋翼盘 Y 跨度:", round(span))
    print("结论:", "机体水平/展布正常" if span > 60 else "机体可能侧躺(盘堆叠)")
