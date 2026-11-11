"""Check rendered blink phases against image source time, including sampling gaps."""
import math


def blink_evidence(samples, channel):
    interior=[s for s in samples if .1<(s['stamp']%1.)<.9]
    wrong=[s['stamp'] for s in interior if (s['dominant_rgb_pixels'][channel]>10)!=(s['stamp']%2.<1.)]
    phases={int(math.floor(s['stamp'])) for s in interior}
    cycles=[p//2 for p in phases if p%2==0 and p+1 in phases]
    gaps=[b['stamp']-a['stamp'] for a,b in zip(samples,samples[1:])]
    return dict(passed=not wrong and len(cycles)>=3,wrong_phase_source_stamps=wrong,
                complete_sampled_cycles=cycles,max_source_gap_s=max(gaps,default=None),
                evaluated_interior_frames=len(interior),boundary_guard_s=.1,
                criterion='Pixels must match 1 s on / 1 s off at image source time, with interior samples in both phases of at least three cycles. Gaps are not treated as measured transition times.')
