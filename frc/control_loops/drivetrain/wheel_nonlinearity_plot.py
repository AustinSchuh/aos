#!/usr/bin/python3
#
# This is a quick script to show the effect of the wheel nonlinearity term on
# turning rate

from frc.control_loops.python import matplotlib_backend

matplotlib_backend.use_interactive()
from matplotlib import pylab
import numpy

if __name__ == '__main__':
    x = numpy.arange(-1, 1, 0.01)

    for nonlin in numpy.arange(0.2, 1.0, 0.1):
        angular_range = numpy.pi * nonlin / 2.0
        newx1 = numpy.sin(x * angular_range) / numpy.sin(angular_range)
        newx2 = numpy.sin(newx1 * angular_range) / numpy.sin(angular_range)

        pylab.plot(x, newx2, label="nonlin %f" % nonlin)

    pylab.legend()
    pylab.show()
