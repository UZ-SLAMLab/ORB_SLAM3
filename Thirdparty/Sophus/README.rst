linux, os x: |TravisCI|_ windows: |AppVeyor|_ code coverage: |ci_cov|_


Sophus
======

Overview
--------

This is a c++ implementation of Lie groups commonly used for 2d and 3d
geometric problems (i.e. for Computer Vision or Robotics applications).
Among others, this package includes the special orthogonal groups SO(2) and
SO(3) to present rotations in 2d and 3d as well as the special Euclidean group
SE(2) and SE(3) to represent rigid body transformations (i.e. rotations and
translations) in 2d and 3d.

API documentation: https://strasdat.github.io/Sophus/

Cross platform support
----------------------

Sophus compiles with clang and gcc on Linux and OS X as well as msvc on Windows.
The specific compiler and operating system versions which are supported are
the ones which are used in the Continuous Integration (CI): See TravisCI_ and
AppVeyor_ for details.

However, it should work (with no to minor modification) on many other
modern configurations as long they support c++11, CMake, and Eigen 3.X.

.. |TravisCI| image:: https://travis-ci.org/strasdat/Sophus.svg?branch=master
.. _TravisCI: https://travis-ci.org/strasdat/Sophus

.. |AppVeyor| image:: https://ci.appveyor.com/api/projects/status/um4285lwhs8ci7pt/branch/master?svg=true
.. _AppVeyor: https://ci.appveyor.com/project/strasdat/sophus/branch/master

.. |ci_cov| image:: https://coveralls.io/repos/github/strasdat/Sophus/badge.svg?branch=master
.. _ci_cov: https://coveralls.io/github/strasdat/Sophus?branch=master
