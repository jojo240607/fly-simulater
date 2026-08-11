//! 构建脚本：链接预编译的 `phy_ffi` C-ABI 物理引擎库。
//!
//! 物理引擎以 cdylib 形态集成（非源码依赖）：库与头文件位于
//! `d:/project/game/physics/pkg/release/`。因 `physics` 仓库的 `pkg/`
//! 被 gitignore，这里用绝对路径指向源工程已发布的 release 产物。
//!
//! 链接方式（Windows MSVC）：`phy_ffi.lib`（导入库）+ 运行时 `phy_ffi.dll`。
//! 若换 MinGW，则链接 `libphy_ffi.dll.a`。

use std::fs;

fn main() {
    // 物理引擎发布库目录（与 DESIGN.md §3.1 一致）。
    let phy_pkg = "d:/project/game/physics/pkg/release";

    println!("cargo:rustc-link-search=native={}", phy_pkg);
    // MSVC 导入库名（不带前缀 lib / 扩展名）。
    println!("cargo:rustc-link-lib=phy_ffi");

    // 把运行时 DLL 复制到 exe 输出目录（target/<profile>/），否则运行时
    // STATUS_DLL_NOT_FOUND。OUT_DIR = target/<profile>/build/<pkg>-<hash>/out，
    // 上溯 3 级得到 target/<profile>。
    if let Ok(out_dir) = std::env::var("OUT_DIR") {
        if let Some(target_dir) = std::path::Path::new(&out_dir)
            .ancestors()
            .nth(3)
        {
            let dll_src = std::path::Path::new(phy_pkg).join("phy_ffi.dll");
            let dll_dst = target_dir.join("phy_ffi.dll");
            if dll_src.exists() {
                let _ = fs::copy(&dll_src, &dll_dst);
            }
        }
    }

    // 让 cargo 在库目录变化时重链。
    println!("cargo:rerun-if-changed={}/phy_ffi.lib", phy_pkg);
    println!("cargo:rerun-if-changed={}/phy_ffi.dll", phy_pkg);
}
