# timing_report.py
"""
Utility to analyze and plot timing measurements from apriltags timing CSV files.
"""
import sys
import os
import pandas as pd
import matplotlib.pyplot as plt

def main(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}")
        sys.exit(1)
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} timing measurements from {csv_path}\n")
    print("Timing Report:")
    cdf_stats = {}
    for col in df.columns:
        stats = df[col].describe()
        cdf_stats[col] = {
            'p95': df[col].quantile(0.95),
            'p99': df[col].quantile(0.99)
        }
        print(f"{col:22} min={stats['min']:.0f} max={stats['max']:.0f} mean={stats['mean']:.0f} std={stats['std']:.0f}")
    print("\nCDF Percentile Report:")
    for col in df.columns:
        print(f"  95% of {col:22} measurements are below {cdf_stats[col]['p95']:.0f} us (CDF)")
        print(f"  99% of {col:22} measurements are below {cdf_stats[col]['p99']:.0f} us (CDF)")
    print("\nGenerating plots...")
    for col in df.columns:
        # Line plot
        plt.figure()
        plt.plot(df[col], label=col)
        plt.title(f"{col} over time")
        plt.xlabel("Frame")
        plt.ylabel("Microseconds")
        plt.legend()
        plt.tight_layout()
        plt.savefig(f"{col}_plot.png")
        plt.close()
        print(f"Saved plot: {col}_plot.png")
        # Histogram plot
        plt.figure()
        plt.hist(df[col], bins=256, color='skyblue', edgecolor='black')
        plt.title(f"Histogram of {col}")
        plt.xlabel("Microseconds")
        plt.ylabel("Count")
        plt.tight_layout()
        plt.savefig(f"{col}_hist.png")
        plt.close()
        print(f"Saved histogram: {col}_hist.png")
        # CDF plot
        plt.figure()
        sorted_vals = df[col].sort_values()
        cdf = sorted_vals.rank(method='average', pct=True)
        plt.plot(sorted_vals, cdf, label=f"CDF of {col}")
        # Percentile statistics
        p95 = sorted_vals.quantile(0.95)
        p99 = sorted_vals.quantile(0.99)
        # Add vertical lines and labels
        plt.axvline(p95, color='red', linestyle='--', label=f'95% ({p95:.0f} us)')
        plt.axvline(p99, color='orange', linestyle='--', label=f'99% ({p99:.0f} us)')
        plt.title(f"CDF of {col}")
        plt.xlabel("Microseconds")
        plt.ylabel("Fraction of measurements")
        plt.legend()
        plt.tight_layout()
        plt.savefig(f"{col}_cdf.png")
        plt.close()
        print(f"Saved CDF plot: {col}_cdf.png")
    print("\nDone.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <timing_csv_file>")
        sys.exit(1)
    main(sys.argv[1])
