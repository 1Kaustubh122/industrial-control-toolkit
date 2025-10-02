#!/usr/bin/env python3
import sys
import pathlib
import pandas as pd
import matplotlib.pyplot as plt

def plot_pid_csv(csv_path: pathlib.Path):
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots(figsize=(10,6))
    if "u" in df: ax.plot(df["t_ms"], df["u"], label="Control u")
    if "v" in df: ax.plot(df["t_ms"], df["v"], label="Speed v")
    if "x" in df: ax.plot(df["t_ms"], df["x"], label="Position x")
    if "y" in df: ax.plot(df["t_ms"], df["y"], label="Output y")
    if "r" in df: ax.plot(df["t_ms"], df["r"], "--", label="Reference r")

    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Value")
    ax.set_title(csv_path.stem)
    ax.legend()
    ax.grid(True)
    
    out_dir = str(csv_path.parent.parent) + "/plots/" + (f"{csv_path.stem}")
    out_dir = pathlib.Path(out_dir)
    # out_dir.mkdir(parents=True, exist_ok=True)
        
    out_path = out_dir.with_suffix(".png")
  
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_pid.py <csvfile>")
        sys.exit(1)

    csv = pathlib.Path(sys.argv[1]).resolve()
    plot_pid_csv(csv)
