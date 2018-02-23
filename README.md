ARy - Augmented Reality library
===============================

ARy is a library that provides anchor tracking algorithms which can be used to develop various augmented reality applications. The library is at the moment of a more demonstrative nature, but hopefully it will someday mature into a serious AR project.

Buidling
--------

The library is compiled using CMake. At the moment it depends only on the OpenCV library (should work with OpenCV 2.4 and 3.x).

Godot
-----

There is a Godot plugin availabe that enables integration of ARy into the game engine using GDNative API. At the moment the AR is only partially supported in Godot (version 3.0) so some bypasses had to be done to bring camera image to the background. A demo of a project will be added at some point.
