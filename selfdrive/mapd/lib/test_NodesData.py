import unittest
import numpy as np
from numpy.testing import assert_array_almost_equal
from selfdrive.mapd.lib.mock_data import MockRoad
from selfdrive.mapd.lib.NodesData import vectors


class TestNodesData(unittest.TestCase):
  def test_vectors(self):
    points = np.radians(MockRoad.road1_points_grad)
    expected = np.array([
        [-1.34011951e-05, 1.00776468e-05],
        [-5.83610920e-06, 4.41046897e-06],
        [-7.83348567e-06, 5.94114032e-06],
        [-7.08560788e-06, 5.30408795e-06],
        [-6.57632550e-06, 4.05791838e-06],
        [-1.16077872e-06, 6.91151252e-07],
        [-1.53178098e-05, 9.62215139e-06],
        [-5.76314175e-06, 3.55176643e-06],
        [-1.61124141e-05, 9.86127759e-06],
        [-1.48006628e-05, 8.58192512e-06],
        [-1.72237209e-06, 1.60570482e-06],
        [-8.68985228e-06, 9.22062311e-06],
        [-1.42922812e-06, 1.51494711e-06],
        [-3.39761486e-06, 2.57087743e-06],
        [-2.75467373e-06, 1.28631255e-06],
        [-1.57501989e-05, 5.72309451e-06],
        [-2.52143954e-06, 1.34565295e-06],
        [-1.65278643e-06, 1.28630942e-06],
        [-2.22196114e-05, 1.64360838e-05],
        [-5.88675934e-06, 4.08234746e-06],
        [-1.83673390e-06, 1.46782408e-06],
        [-1.55004206e-06, 1.51843800e-06],
        [-1.20451533e-06, 2.06298011e-06],
        [-1.91801338e-06, 4.64083285e-06],
        [-2.38653483e-06, 5.60076524e-06],
        [-1.65269781e-06, 5.78402290e-06],
        [-3.66908309e-07, 2.75412965e-06],
        [0.00000000e+00, 1.92858882e-06],
        [9.09242615e-08, 2.66162711e-06],
        [3.14490354e-07, 1.53065382e-06],
        [8.66452477e-08, 4.83456208e-07],
        [2.41750593e-07, 1.10828411e-06],
        [7.43745228e-06, 1.27618831e-05],
        [5.59968054e-06, 9.63947367e-06],
        [2.01951467e-06, 2.75413219e-06],
        [4.59952643e-07, 6.42281301e-07],
        [1.74353749e-06, 1.74533121e-06],
        [2.57144338e-06, 2.11185266e-06],
        [1.46893187e-05, 1.11999169e-05],
        [3.84659229e-05, 2.85527952e-05],
        [2.71627936e-05, 1.98727946e-05],
        [8.44632540e-06, 6.15058628e-06],
        [2.29420323e-06, 1.92859222e-06],
        [2.58083439e-06, 3.16952222e-06],
        [3.76373643e-06, 5.14174911e-06],
        [5.32416098e-06, 6.51707770e-06],
        [8.62890928e-06, 1.11998258e-05],
        [1.25762497e-05, 1.65231340e-05],
        [8.90452991e-06, 1.10148240e-05],
        [4.86505726e-06, 4.59023120e-06],
        [3.85545276e-06, 3.39642031e-06],
        [3.48753893e-06, 3.30566145e-06],
        [2.99557303e-06, 2.61276368e-06],
        [2.15496788e-06, 1.87797727e-06],
        [4.10564937e-06, 3.58142649e-06],
        [1.53680853e-06, 1.33866906e-06],
        [4.99540175e-06, 4.35635790e-06],
        [1.37744970e-06, 1.19380643e-06],
        [1.74319821e-06, 1.28456429e-06],
        [9.99931238e-07, 1.14493663e-06],
        [6.42735560e-07, 1.19380547e-06],
        [3.66818436e-07, 1.46782199e-06],
        [5.45413874e-08, 1.83783170e-06],
        [-1.35818548e-07, 1.14842666e-06],
        [-5.50758101e-07, 3.02989178e-06],
        [-4.58785270e-07, 2.66162724e-06],
        [-2.51315555e-07, 1.19031459e-06],
        [-3.91409773e-07, 1.65457223e-06],
        [-2.14525206e-06, 5.67755902e-06],
        [-4.24558096e-07, 1.39102753e-06],
        [-1.46936730e-06, 5.32325561e-06],
        [-1.37632061e-06, 4.59021715e-06],
        [-8.26642899e-07, 4.68097349e-06],
        [-6.42702724e-07, 4.95673534e-06],
        [-3.66796960e-07, 7.25009780e-06],
        [-1.82861669e-07, 8.99542699e-06],
        [4.09564134e-07, 6.11214315e-06],
        [7.80629912e-08, 1.45734993e-06],
        [4.81205526e-07, 7.56076647e-06],
        [2.01036346e-07, 2.42775302e-06]])

    v = vectors(points)
    assert_array_almost_equal(v, expected)
