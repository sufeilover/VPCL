# Offline reproduction

Run `python reproduce.py section4`, `case1`, `case2`, or `human` from the repository root. `all` runs these four tasks in sequence. Use `--output PATH` for another output directory.

Section IV uses the packaged 333-Hz aggregate curves, with representative horizons 125, 250 and 500. The fixed exclusion of Silverstone/Supra AIpush100 rollouts 172, 173 and 174 is inherited from the existing analysis; this package does not select new exclusions. There are 5,718 retained rollouts in the source analysis. Means and empirical P95 are not interchangeable.

The three-layer analysis reports physical-unit errors, configuration-specific relative growth, and preference-dependent distance/error utility. Lambda is a preference parameter, not an accuracy threshold or a safety guarantee. Equal group weights avoid counting the two geometry metrics as two complete state groups. The portable task redraws results; it does not rerun original AC/model alignment from the large Excel/rollout tables.

The runtime source scripts and the archived preprocessing scripts are kept separate from the portable runner. The runner changes only path bindings and the unavailable Windows italic-font dependency in the drawing code; mathematical processing is retained. ReportLab's bundled Vera italic font is used for portable lambda glyphs, so output PDFs need not be byte-identical to the paper's PDFs.

Case 1 rechecks request-level 80/160/240-step persistence summaries. Case 2 recomputes unpaired group summaries from whole-circuit inputs with five AI and seven assisted runs per AIpush. Human summaries use three runs per participant/condition; AI and human distances use different original reference-line implementations and should not be pooled.

Timing files include stage summaries and processed selected rows. Raw transport logs are not included; the original timing analyzers document how these results were produced. AIpush condition summaries are included without raw driving workbooks. Full raw-to-result reproduction requires those external inputs and confirmed runtime versions.
