import unittest

from solver_ipft import PyConversionTest
from solver_ipft import PyPlannerCld
from solver_ipft import PyParticleBelief
from solver_ipft import PyNode, PyVNode, PyQNode

import numpy as np


class conversionTest(unittest.TestCase):
    pyct = PyConversionTest()

    def testPointConv(self):
        arr = np.array([10.])
        self.assertAlmostEqual(arr, self.pyct.testPointConv())

    def testActionConv(self):
        arr = np.array([-3.])
        self.assertAlmostEqual(arr, self.pyct.testActionConv())

    def testValueConv(self):
        arr = np.array([20., 1.])
        self.assertAlmostEqual(arr[0], self.pyct.testValueConv()[0])
        self.assertAlmostEqual(arr[1]*60, self.pyct.testValueConv()[1])

    def testParticleBeliefConv(self):
        bel = self.pyct.testParticleBeliefConv()
        print("Particles:")
        print(bel.particles_)
        print(bel.weights_)
        print("Posterior Particles:")
        print(bel.post_particles_)
        print(bel.post_weights_)

    def testTreeConv(self):
        print("test tree conversion:")
        pyroot = self.pyct.testTreeConv()
        print(pyroot.count_)
        print(pyroot.treelevel_)
        print(pyroot.values_)
        print(pyroot.belief_.particles_)
        self.assertAlmostEqual(4.253895, pyroot.belief_.particles_[0][0])
        print(pyroot.belief_.weights_)
        print(pyroot.children_)
        print(pyroot.children_[0].action_[0])
        print(pyroot.children_[3].values_)
        self.assertAlmostEqual(-3., pyroot.children_[0].action_[0])
        print(pyroot.children_[0].children_)

    def testNpParticleArrayConv(self):
        arr = np.array([[5.],
                        [2.]])
        self.assertTrue(self.pyct.testNpParticleArrayConv(arr))


class PyPlannerCldTest(unittest.TestCase):
    def testConstructor(self):
        p = PyPlannerCld()


if __name__ == '__main__':
    unittest.main()
