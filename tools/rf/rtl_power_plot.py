"""Plot an rtl_power CSV: waterfall + mean/max spectrum, and print a summary.
usage: rtl_power_plot.py in.csv out.png"""
import sys
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = defaultdict(dict)  # time -> {freq: dB}
for line in open(sys.argv[1]):
    p = [x.strip() for x in line.split(",")]
    t = p[0] + " " + p[1]
    lo, step = float(p[2]), float(p[4])
    for i, v in enumerate(p[6:]):
        rows[t][lo + i * step] = float(v)
times = sorted(rows)
freqs = np.array(sorted({f for r in rows.values() for f in r}))
grid = np.array([[rows[t].get(f, np.nan) for f in freqs] for t in times])
mean, mx = np.nanmean(grid, 0), np.nanmax(grid, 0)
floor = np.nanmedian(mean)
print(f"{len(times)} sweeps, {len(freqs)} bins, {freqs[0]/1e6:.3f}-{freqs[-1]/1e6:.3f} MHz")
print(f"noise floor (median of mean) {floor:.1f} dB; mean range {np.nanmin(mean):.1f}..{np.nanmax(mean):.1f}; peak max {np.nanmax(mx):.1f} dB")
hot = mx > floor + 10
print(f"bins with a burst >10 dB over floor: {hot.sum()}")
for f, m, a in sorted(zip(freqs[hot], mx[hot], mean[hot]), key=lambda x: -x[1])[:10]:
    print(f"  {f/1e6:8.3f} MHz  max {m:6.1f}  mean {a:6.1f}")
fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True, gridspec_kw=dict(height_ratios=[2, 1]))
a1.imshow(grid, aspect="auto", origin="upper", extent=[freqs[0]/1e6, freqs[-1]/1e6, len(times), 0], cmap="viridis")
a1.set_ylabel("sweep (1 s each)"); a1.set_title(sys.argv[1].split("/")[-1])
a2.plot(freqs/1e6, mean, label="mean"); a2.plot(freqs/1e6, mx, label="max", alpha=0.7)
a2.set_xlabel("MHz"); a2.set_ylabel("dB (uncalibrated)"); a2.legend(); a2.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(sys.argv[2], dpi=110)
