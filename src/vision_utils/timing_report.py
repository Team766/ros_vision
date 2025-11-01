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
    # Prepare markdown report filename
    base_name = os.path.splitext(os.path.basename(csv_path))[0]
    md_report = f"{base_name}_report.md"
    with open(md_report, "w") as md:
        md.write(f"# Timing Report for `{csv_path}`\n\n")
        md.write(f"Loaded **{len(df)}** timing measurements from `{csv_path}`\n\n")
        md.write("## Summary Statistics\n\n")
        md.write("| Metric | Min | Max | Mean | Std |\n")
        md.write("|--------|-----|-----|------|-----|\n")
        for col in df.columns:
            stats = df[col].describe()
            md.write(f"| {col} | {stats['min']:.0f} | {stats['max']:.0f} | {stats['mean']:.0f} | {stats['std']:.0f} |\n")
        md.write("\n## CDF Percentile Report\n\n")
        md.write("| Metric | 95% below (us) | 99% below (us) |\n")
        md.write("|--------|----------------|----------------|\n")
        for col in df.columns:
            md.write(f"| {col} | {cdf_stats[col]['p95']:.0f} | {cdf_stats[col]['p99']:.0f} |\n")
        md.write("\n## Plots\n\n")
    print(f"\nGenerating plots and markdown report: {md_report}\n")
    for col in df.columns:
        # Line plot
        plt.figure()
        plt.plot(df[col], label=col)
        plt.title(f"{col} over time")
        plt.xlabel("Frame")
        plt.ylabel("Microseconds")
        plt.legend()
        plt.tight_layout()
        line_plot = f"{col}_plot.png"
        plt.savefig(line_plot)
        plt.close()
        print(f"Saved plot: {line_plot}")
        # Histogram plot
        plt.figure()
        plt.hist(df[col], bins=256, color='skyblue', edgecolor='black')
        plt.title(f"Histogram of {col}")
        plt.xlabel("Microseconds")
        plt.ylabel("Count")
        plt.tight_layout()
        hist_plot = f"{col}_hist.png"
        plt.savefig(hist_plot)
        plt.close()
        print(f"Saved histogram: {hist_plot}")
        # CDF plot
        plt.figure()
        sorted_vals = df[col].sort_values()
        cdf = sorted_vals.rank(method='average', pct=True)
        plt.plot(sorted_vals, cdf, label=f"CDF of {col}")
        p95 = sorted_vals.quantile(0.95)
        p99 = sorted_vals.quantile(0.99)
        plt.axvline(p95, color='red', linestyle='--', label=f'95% ({p95:.0f} us)')
        plt.axvline(p99, color='orange', linestyle='--', label=f'99% ({p99:.0f} us)')
        plt.title(f"CDF of {col}")
        plt.xlabel("Microseconds")
        plt.ylabel("Fraction of measurements")
        plt.legend()
        plt.tight_layout()
        cdf_plot = f"{col}_cdf.png"
        plt.savefig(cdf_plot)
        plt.close()
        print(f"Saved CDF plot: {cdf_plot}")
        # Add images to markdown
        with open(md_report, "a") as md:
            md.write(f"### {col}\n\n")
            md.write(f"**Line Plot:**\n\n![]({line_plot})\n\n")
            md.write(f"**Histogram:**\n\n![]({hist_plot})\n\n")
            md.write(f"**CDF Plot:**\n\n![]({cdf_plot})\n\n")
    print(f"\nMarkdown report saved to: {md_report}\n")
    # Optionally generate PDF from markdown using pandoc
    pdf_report = f"{base_name}_report.pdf"
    try:
        import subprocess
        subprocess.run([
            "pandoc", md_report, "-o", pdf_report
        ], check=True)
        print(f"PDF report saved to: {pdf_report}")
    except Exception as e:
        print(f"Could not generate PDF automatically. To create PDF manually, run:")
        print(f"pandoc {md_report} -o {pdf_report}")
    print("Done.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <timing_csv_file>")
        sys.exit(1)
    main(sys.argv[1])
