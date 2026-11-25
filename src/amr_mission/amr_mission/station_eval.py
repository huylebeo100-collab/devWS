#!/usr/bin/env python3
"""Tổng hợp sai số tại trạm từ station_errors.csv (do mission_manager ghi).

Trong container (đã build package):
  ros2 run amr_mission station_eval                    # đọc /root/amr_eval/station_errors.csv
  ros2 run amr_mission station_eval --last             # chỉ session mới nhất
Trên laptop (không cần ROS, chỉ cần matplotlib):
  python3 station_eval.py station_errors.csv
Tùy chọn: --session 20261120_103000  --tol-xy 0.06 --tol-yaw 0.10  --order A,B,C,home  --out <thư mục>

Ra (cùng thư mục với CSV, hoặc --out):
  station_summary.txt   bảng tổng hợp
  station_summary.csv   bảng tổng hợp dạng CSV
  sai_so_tai_tram.png   biểu đồ (cột = trung bình theo trạm)
"""
import argparse
import csv
import math
import os
import sys


def load(path, session=None, last=False):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("CSV trống")
    if last:
        session = max(r["session"] for r in rows)
    if session:
        rows = [r for r in rows if r["session"] == session]
        if not rows:
            sys.exit(f"không có dòng nào của session {session}")
    return rows, session


def station_order(rows, order_arg):
    if order_arg:
        return [s.strip() for s in order_arg.split(",") if s.strip()]
    seen = []
    for r in rows:
        if r["station"] not in seen:
            seen.append(r["station"])
    if "home" in seen:          # home để cuối
        seen.remove("home")
        seen.append("home")
    return seen


def summarize(rows, stations, tol_xy_cm, tol_yaw_deg):
    out = []
    for st in stations:
        rs = [r for r in rows if r["station"] == st]
        if not rs:
            continue
        exy = [float(r["e_xy_cm"]) for r in rs]
        eyaw = [float(r["e_yaw_deg"]) for r in rs]
        ok = sum(1 for a, b in zip(exy, eyaw) if a <= tol_xy_cm and abs(b) <= tol_yaw_deg)
        out.append({
            "station": st, "n": len(rs),
            "e_xy_mean_cm": sum(exy) / len(exy), "e_xy_max_cm": max(exy),
            "e_yaw_mean_deg": sum(eyaw) / len(eyaw), "e_yaw_maxabs_deg": max(abs(v) for v in eyaw),
            "pass": ok, "exy": exy, "eyaw": eyaw,
        })
    return out


def write_summary(summ, folder, title, tol_xy_cm, tol_yaw_deg):
    lines = [title, f"Dung sai: vị trí {tol_xy_cm:.0f} cm, góc {tol_yaw_deg:.1f}°", "",
             f"{'Trạm':<8}{'Số lần':>7}{'e_xy TB':>10}{'e_xy max':>10}{'e_yaw TB':>10}"
             f"{'|e_yaw| max':>13}{'Đạt':>8}",
             f"{'':<8}{'':>7}{'(cm)':>10}{'(cm)':>10}{'(độ)':>10}{'(độ)':>13}{'':>8}"]
    for s in summ:
        lines.append(f"{s['station']:<8}{s['n']:>7}{s['e_xy_mean_cm']:>10.1f}{s['e_xy_max_cm']:>10.1f}"
                     f"{s['e_yaw_mean_deg']:>+10.1f}{s['e_yaw_maxabs_deg']:>13.1f}"
                     f"{s['pass']:>5}/{s['n']:<2}")
    allxy = [v for s in summ for v in s["exy"]]
    allyaw = [v for s in summ for v in s["eyaw"]]
    npass = sum(s["pass"] for s in summ)
    lines += ["", f"Tất cả: {len(allxy)} lần tới trạm, e_xy TB {sum(allxy) / len(allxy):.1f} cm, "
                  f"max {max(allxy):.1f} cm; |e_yaw| max {max(abs(v) for v in allyaw):.1f}°; "
                  f"đạt dung sai {npass}/{len(allxy)}"]
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(folder, "station_summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    with open(os.path.join(folder, "station_summary.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["station", "n", "e_xy_mean_cm", "e_xy_max_cm", "e_yaw_mean_deg",
                    "e_yaw_maxabs_deg", "pass"])
        for s in summ:
            w.writerow([s["station"], s["n"], f"{s['e_xy_mean_cm']:.1f}", f"{s['e_xy_max_cm']:.1f}",
                        f"{s['e_yaw_mean_deg']:.1f}", f"{s['e_yaw_maxabs_deg']:.1f}", s["pass"]])


def plot(summ, folder, tol_xy_cm, tol_yaw_deg):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nKhông có matplotlib: bỏ qua biểu đồ (chép CSV ra laptop rồi chạy lại, "
              "hoặc pip3 install matplotlib)")
        return

    names = [s["station"] for s in summ]
    ep = [s["e_xy_mean_cm"] for s in summ]
    eg = [s["e_yaw_mean_deg"] for s in summ]
    c_pos, c_yaw = "tab:blue", "tab:orange"
    wb = 0.36
    idx = list(range(len(names)))

    fig, a1 = plt.subplots(figsize=(9, 4.5))
    a2 = a1.twinx()
    L1 = max(max(ep), tol_xy_cm) * 1.3                # đối xứng -> 2 trục chung vạch 0
    L2 = max(max(abs(v) for v in eg), tol_yaw_deg) * 1.3
    a1.set_ylim(-L1, L1)
    a2.set_ylim(-L2, L2)
    a1.set_xlim(-0.6, len(names) - 0.4)

    a1.bar([i - wb / 2 for i in idx], ep, wb, color=c_pos, label="Sai số vị trí (cm)")
    a2.bar([i + wb / 2 for i in idx], eg, wb, color=c_yaw, label="Sai số góc (độ)")
    a1.axhline(0, color="k", lw=0.8)

    for v in (tol_xy_cm, -tol_xy_cm):
        a1.axhline(v, color=c_pos, ls="--", lw=1)
        a1.text(-0.55, v, f"{v:+.0f} cm", color=c_pos, fontsize=8, ha="left",
                va="bottom" if v > 0 else "top")
    for v in (tol_yaw_deg, -tol_yaw_deg):
        a2.axhline(v, color=c_yaw, ls=":", lw=1.3)
        a2.text(len(names) - 0.45, v, f"{v:+.1f}°", color=c_yaw, fontsize=8, ha="right",
                va="bottom" if v > 0 else "top")

    for i in idx:
        a1.text(i - wb / 2, ep[i], f"{ep[i]:.1f}", ha="center", va="bottom", fontsize=9)
        a2.text(i + wb / 2, eg[i], f"{eg[i]:+.1f}", ha="center",
                va="bottom" if eg[i] >= 0 else "top", fontsize=9)

    a1.set_xticks(idx)
    a1.set_xticklabels(names)
    a1.set_ylabel("Sai số vị trí (cm)", color=c_pos)
    a2.set_ylabel("Sai số góc (độ)", color=c_yaw)
    a1.yaxis.grid(True, alpha=0.3)
    a1.set_axisbelow(True)
    a1.set_title("Sai số tại trạm", pad=26)
    h1, l1 = a1.get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    a1.legend(h1 + h2, l1 + l2, loc="lower left", bbox_to_anchor=(0.0, 1.0),
              ncol=2, frameon=False, fontsize=9)
    fig.tight_layout()
    out = os.path.join(folder, "sai_so_tai_tram.png")
    fig.savefig(out, dpi=200)
    print(f"\nĐã lưu {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default="/root/amr_eval/station_errors.csv")
    ap.add_argument("--session")
    ap.add_argument("--last", action="store_true", help="chỉ session mới nhất")
    ap.add_argument("--tol-xy", type=float, default=0.06, help="m, khớp xy_goal_tolerance")
    ap.add_argument("--tol-yaw", type=float, default=0.10, help="rad, khớp yaw_goal_tolerance")
    ap.add_argument("--order", help="thứ tự trạm trên biểu đồ, vd A,B,C,home")
    ap.add_argument("--out", help="thư mục lưu kết quả (mặc định: cạnh file CSV)")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    if not os.path.exists(a.csv):
        sys.exit(f"không thấy {a.csv} (mission_manager chưa ghi lần nào tới trạm?)")
    rows, session = load(a.csv, a.session, a.last)
    folder = a.out or os.path.dirname(os.path.abspath(a.csv))
    os.makedirs(folder, exist_ok=True)
    tol_xy_cm, tol_yaw_deg = a.tol_xy * 100, math.degrees(a.tol_yaw)
    summ = summarize(rows, station_order(rows, a.order), tol_xy_cm, tol_yaw_deg)
    if not summ:
        sys.exit("không có trạm nào khớp --order")
    sessions = sorted({r["session"] for r in rows})
    title = f"Sai số tại trạm | session: {session or ', '.join(sessions)}"
    write_summary(summ, folder, title, tol_xy_cm, tol_yaw_deg)
    if not a.no_plot:
        plot(summ, folder, tol_xy_cm, tol_yaw_deg)


if __name__ == "__main__":
    main()