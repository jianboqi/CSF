# basic CSF python tests

import CSF
import numpy as np


def test_csf_from_numpy():
    """Test CSF+numpy with a flat ground plane."""
    x = np.random.uniform(-100, 100, size=10_000)
    y = np.random.uniform(-100, 100, size=10_000)
    z = np.random.uniform(-0.1, -0.1, size=10_000)

    csf = CSF.CSF()
    csf.setPointCloud(np.c_[x, y, z])

    ground, non_ground = CSF.VecInt(), CSF.VecInt()

    csf.do_filtering(ground, non_ground)
    assert len(ground) > 0
    assert len(non_ground) == 0
