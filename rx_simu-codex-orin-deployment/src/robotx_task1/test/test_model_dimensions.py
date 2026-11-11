"""Check assembled geometry against the official cut-panel dimensions."""
import math
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[3]


def vertices(name):
    path=ROOT/'models/robotx_2026_robobuoy/meshes'/name
    return [list(map(float,line.split()[1:])) for line in path.read_text().splitlines() if line.startswith('vertex ')]


class Dimensions(unittest.TestCase):
    def test_cut_panel_dimensions_survive_assembly(self):
        a,b,c=vertices('panels.stl')[:3]
        self.assertAlmostEqual(math.dist(a,b),17*.0254)
        # Distance between parallel top/bottom edge midpoints is the cut height.
        midpoint=[(a[i]+b[i])/2 for i in range(3)]
        top=[0.,c[1],c[2]]
        self.assertAlmostEqual(math.dist(midpoint,top),12*.0254)
        self.assertAlmostEqual(abs(c[0])*2,7.25*.0254)

    def test_black_hat_and_pipe_dimensions(self):
        hat=vertices('sunhat.stl');pipe=vertices('beacon_pipe.stl')
        self.assertAlmostEqual(max(math.hypot(p[0],p[1]) for p in hat)*2,10*.0254)
        self.assertAlmostEqual(max(math.hypot(p[0],p[1]) for p in pipe)*2,3.5*.0254)
        self.assertAlmostEqual(max(p[2] for p in pipe)-min(p[2] for p in pipe),5.25*.0254)


if __name__=='__main__':unittest.main()
