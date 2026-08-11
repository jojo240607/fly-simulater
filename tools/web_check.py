"""无头浏览器验证 Web 仿真 UI：打开页面、等待帧、检查 canvas 非空、读遥测、截图。
用法：python tools/web_check.py [url]
"""
import sys
import time
from playwright.sync_api import sync_playwright

URL = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8080/"


def main():
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--no-sandbox", "--use-gl=swiftshader"])
        page = browser.new_page(viewport={"width": 1100, "height": 700})
        errors = []
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(URL, wait_until="load", timeout=15000)

        # 等待 WS 连接 + 至少收到一帧：轮询 canvas 非黑像素数。
        t0 = time.time()
        got_frame = False
        while time.time() - t0 < 12:
            try:
                cnt = page.evaluate(
                    """() => {
                        const c = document.getElementById('view');
                        const g = c.getContext('2d');
                        const d = g.getImageData(0,0,c.width,c.height).data;
                        let n=0;
                        for (let i=0;i<d.length;i+=4){ if(d[i]>10||d[i+1]>10||d[i+2]>10) n++; }
                        return n;
                    }"""
                )
                if cnt > 500:
                    got_frame = True
                    break
            except Exception:
                pass
            time.sleep(0.3)

        status = page.evaluate("() => document.getElementById('status').textContent")
        alt = page.evaluate("() => document.getElementById('t-alt').textContent")
        spd = page.evaluate("() => document.getElementById('t-spd').textContent")
        state = page.evaluate("() => document.getElementById('t-state').textContent")

        # 通过页面 JS 触发场景切换（走真实控制下发路径）。
        page.evaluate(
            "document.getElementById('scenario').value='degraded'; "
            "document.getElementById('scenario').dispatchEvent(new Event('change'));"
        )
        time.sleep(3)
        state2 = page.evaluate("() => document.getElementById('t-state').textContent")

        # 先打印结果（截图失败也不影响）
        print("=== Web UI 检查结果 ===")
        print("got_frame(non-black pixels):", got_frame)
        print("status:", repr(status))
        print("telemetry alt:", repr(alt), "| speed:", repr(spd), "| state(hover):", repr(state))
        print("state after degraded 3s:", repr(state2))
        print("console errors:", errors[:5])
        ok = got_frame and status.strip().startswith("已连接")
        print("RESULT:", "PASS" if ok else "FAIL")

        # 截图（字体加载可能卡住）：优先整页截图，失败则导出 canvas 纯画面。
        try:
            page.screenshot(path="web_check.png", timeout=8000)
            print("screenshot: web_check.png OK")
        except Exception as e:
            try:
                # 绕过 document.fonts.ready：直接导出 canvas PNG。
                b64 = page.evaluate(
                    "() => document.getElementById('view').toDataURL('image/png')"
                )
                import base64
                open("web_check_canvas.png", "wb").write(base64.b64decode(b64.split(",")[1]))
                print("screenshot: web_check_canvas.png OK (canvas export)")
            except Exception as e2:
                print("screenshot skipped:", type(e).__name__, "/", type(e2).__name__)

        browser.close()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
