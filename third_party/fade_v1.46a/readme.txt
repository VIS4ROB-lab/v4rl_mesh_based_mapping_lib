Fade2D, v1.46a, Januray 14th, 2016
----------------------------------

Note: This Fade version is the same as v1.46 but support for Raspberry PI
has been added. Raspberry PI users: Please give feedback on RPI support,
do you have everything you need now for this platform?



Welcome to the Fade2D Library!

Any feedback is appreciated:

 Geom Software
 Bernhard Kornberger
 C++ Freelancer
 bkorn@geom.at


The present download contains all library versions needed for development 
under 
.) Windows (VS2008, VS2010, VS2012, VS2013, VS2015) 
.) Linux (gcc)
.) Apple (clang)


Getting started
===============

Linux and Apple users: 
----------------------

cd examples_2D   (...or examples_25D)
Edit the  Makefile -> uncomment the DISTRO string that matches your system (Apple, Ubuntu, Fedora, ...) 
make
./allExamples_2D   (...or ./examples_25D)


Windows users: 
--------------

Open the solution file 

examples_2D/vs20xx_exampleProject/examples_2D.sln   ...or
examples_25D/vs20xx_exampleProject/examples_25D.sln   

...and compile. Then open a command line window (WIN+r, cmd.exe),
change to fadeRelease/x64 (or fadeRelease/Win32) and run the
executable there. 


For more information please visit http://www.geom.at/fade2d/html




Directories
===========

.) include_fade2d and include_fade25d
Header files of the two libraries. 

.) Win32 (x64)
This directory contains the *.dll and *.lib files for Windows 32-bit (64-bit) 
and it is the target directory for the executables compiled with Visual Studio.

.) lib_${DISTRO}_${ARCHITECTURE}
The shared libraries (*.so) for Linux developers. They work for a wide range 
of Linux distributions. Commercial users who need support for a certain Linux 
distribution, please get in contact with the author. 

.) examples_2D
Source code of all examples using Fade2D

.) examples_25D
Source code of all examples using Fade2.5D

.) doc
Documentation


Student license
===============

Fade is free of charge for personal non-commercial scientific research. But we 
ask you to put a link to Fade on your research homepage or project page and to 
cite Fade2D in scientific publications using it. Fade is not free software. Do
not integrate it in a free software project without explicit permission.

Limits of the student version:
2D points: 1 million
2.5D points: 50000
Meshing: 50000 output triangles


Commercial license
==================

All other applications, including commercial in-house usage, require a commercial 
license which has the advantage of maintenance, error corrections and personal 
support. The commercial license of Fade cosists of a base component and three
optional components:

* The Fade2D base component (mandatory): It computes 2D Delaunay triangulations 
and constrained Delaunay triangulations, i.e. it supports insertion of segments.
* The meshing generator component (optional): Creates high quality triangles 
inside a specified area. The methods refine() and refineAdvanced() refer to
this component.
* The Fade2.5D extension (optional): For terrains and other height fields: 
Points have (x,y,z)-coordinates, ISO lines (contours in a terrain having the 
same height) can be computed and the height values of arbitrary (x,y) pairs
can be queried quickly. The above mentioned meshing extension can take 
advantage of the height values when a height field shall be filled with
high quality triangles.
* The SegmentChecker component with a fast underlying data structure takes
a bunch of line segments and identifies intersecting ones. Visualization
of problem segments and computation of intersection points is supported.

Author: Geom e.U., Graz / Austria 
Bernhard Kornberger, bkorn@geom.at
C++-Freelancer, http://www.geom.at




In no case can Geom Graz/Austria be made responsible for damages of any kind that 
arise in connection with the use or non-usability of the software or information 
provided on our internet pages. If you can't accept these terms, you are not 
allowed to use our software. Using Fade for military research and applications 
is not accepted.


3rd party libraries
-------------------

The Fade 2D library uses the GNU Multiple Precision Arithmetic Library (see 
http://gmplib.org/ for details and the source code), released under the GNU 
Lesser General Public License (https://www.gnu.org/copyleft/lesser.html).



