#!/usr/bin/env python3
"""Ve do thi tu track.csv do circle_track.py sinh ra.

    python3 plot_track.py                  # ve va luu PNG
    python3 plot_track.py --csv track.csv --radius 0.75

Sinh 2 file:
    track_xy.png     quy dao tham chieu va quy dao thuc te
    track_err.png    sai lech theo thoi gian

Can: pip3 install matplotlib
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")           # khong can man hinh, chay duoc qua SSH
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))


def load(path):
    cols = {}
    with open(path) as f:
        r = csv.DictReader(f)
        for k in r.fieldnames:
            cols[k] = []
        for row in r:
            for k, v in row.items():
                cols[k].append(float(v))
    return cols


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(HERE, "track.csv"))
    ap.add_argument("--radius", type=float, default=0.75)
    args = ap.parse_args()

    d = load(args.csv)

    # ---------------------------------------------------- do thi quy dao
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(d["xr"], d["yr"], "b--", lw=1.8, label="quy dao tham chieu")
    ax.plot(d["x"], d["y"], "r-", lw=1.5, label="quy dao thuc te")
    ax.plot(d["x"][0], d["y"][0], "go", ms=9, label="xuat phat")
    ax.plot(d["x"][-1], d["y"][-1], "ks", ms=8, label="ket thuc")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Bam quy dao tron R = {args.radius} m")
    ax.axis("equal")
    ax.grid(True, ls=":", alpha=0.6)
    ax.legend(loc="best")
    out1 = os.path.join(HERE, "track_xy.png")
    fig.tight_layout()
    fig.savefig(out1, dpi=150)

    # ---------------------------------------------------- do thi sai lech
    fig2, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    axs[0].plot(d["t"], [v * 100 for v in d["e_pos"]], "r-", lw=1.3)
    axs[0].set_ylabel("sai lech vi tri (cm)")
    axs[0].grid(True, ls=":", alpha=0.6)

    axs[1].plot(d["t"], [v * 100 for v in d["ex"]], "b-", lw=1.2, label="ex doc truc")
    axs[1].plot(d["t"], [v * 100 for v in d["ey"]], "g-", lw=1.2, label="ey ngang truc")
    axs[1].set_ylabel("sai lech (cm)")
    axs[1].legend(loc="best")
    axs[1].grid(True, ls=":", alpha=0.6)

    axs[2].plot(d["t"], [math.degrees(v) for v in d["eth"]], "m-", lw=1.2)
    axs[2].set_ylabel("sai lech goc (do)")
    axs[2].set_xlabel("thoi gian (s)")
    axs[2].grid(True, ls=":", alpha=0.6)

    fig2.suptitle("Sai lech bam quy dao theo thoi gian")
    out2 = os.path.join(HERE, "track_err.png")
    fig2.tight_layout()
    fig2.savefig(out2, dpi=150)

    # ---------------------------------------------------------- thong ke
    ep = [abs(v) for v in d["e_pos"]]
    er = [abs(v) for v in d["e_radial"]]
    eth = [abs(math.degrees(v)) for v in d["eth"]]
    n = len(ep)
    rmse = math.sqrt(sum(e * e for e in ep) / n)

    print(f"\n  sai lech vi tri   TB {sum(ep)/n*100:6.2f} cm  max {max(ep)*100:6.2f} cm")
    print(f"  RMSE vi tri          {rmse*100:6.2f} cm")
    print(f"  sai lech ban kinh TB {sum(er)/n*100:6.2f} cm  max {max(er)*100:6.2f} cm")
    print(f"  sai lech goc      TB {sum(eth)/n:6.2f} do  max {max(eth):6.2f} do")
    print(f"\n  da luu {out1}")
    print(f"  da luu {out2}\n")


if __name__ == "__main__":
    main()