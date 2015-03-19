Welcome to the OpenSurgSim Project

OpenSurgSim is an open-source project dedicated to real-time surgical
simulation. It offers an open framework that includes the necessary building
blocks for surgical simulations, such as native device support, haptic feedback,
graphics, discrete collision detection and physics simulation. OpenSurgSim is
flexible. Developers can refactor the physics engine, swap models, ODE solvers,
or linear system solvers. For current information and documentation about the
project, please visit our website:

    http://www.opensurgsim.org/

Support can also be found on our public mailing list at
opensurgsim@simquest.com. An archive of the mailing list is at:

    http://groups.google.com/a/simquest.com/group/opensurgsim/

To help get started using OpenSurgSim, use the following quick start guide:

    1. Getting OpenSurgSim
    2. Compiling on GNU/Linux
    3. Compiling on Microsoft Windows
    Appendix: Dependencies


1. Getting OpenSurgSim
======================

OpenSurgSim uses Git for source control, and this is the easiest way to obtain
the most up to date version. Resources for installing and using Git can be found
on Git's website (See Appendix). To obtain OpenSurgSim, run the following
command:

    git clone git://git.assembla.com/OpenSurgSim.git 

This will download the source code for OpenSurgSim and place it in the
OpenSurgSim directory.


2. Compiling on GNU/Linux
=========================

OpenSurgSim has been tested extensively on Debian Testing (Jessie). If you are
using another GNU/Linux distribution, please check your package management
system for the dependencies (see Appendix). For Debian/Ubuntu based systems, the
dependencies are easily installed through apt-get:

    sudo apt-get install libboost-all-dev cmake doxygen libeigen3-dev
    sudo apt-get install google-mock libjs-mathjax libopenscenegraph-dev

To build OpenSurgSim, issue the following commands from the same directory used
to obtain OpenSurgSim (see Section 1)

    mkdir OpenSurgSim/Build
    cd OpenSurgSim/Build
    cmake ../
    make

That's it!


3. Compiling on Microsoft Windows 
=================================

OpenSurgSim has been developed and tested on Microsoft Windows 7, however, other
versions of Windows are likely to work. At least Microsoft Visual Studio 2012 is
required to build OpenSurgSim.

First obtain the source code (Section 1), and install the required dependencies
(Appendix). In addition, the following environment variables will help CMake
automatically find the dependencies, especially if they are not installed to the
default locations.

    BOOST_ROOT  <Path to Boost>
    EIGEN_DIR   <Path to Eigen>
    OSG_ROOT    <Path to OSG>

Also, add %OSG_ROOT%/bin to your PATH environment variable.

Generating the Visual Studio solution file is done with the CMake graphical user
interface. Open CMake, and enter the OpenSurgSim folder (from Section 1) into
the "Where is the source code" field. Then enter a new directory into the "Where
to build the binaries" field. This folder is referred to as BUILD folder for
the purpose of the following steps.

Next, click on the 'Configure' button. This will allow you to select the
compiler (Visual Studio 11 for Visual Studio 2012). If CMake reports “library
not found”, it is likely that your system variables BOOST_ROOT, EIGEN_DIR or
OSG_ROOT are incorrect. Once the configuration is complete without error, click
on the 'Generate' button.

Finally, go to the BUILD folder, open the OpenSurgSim solution file, and build
the ALL_BUILD project.


Appendix: Dependencies
======================

To compile OpenSurgSim, you will need the following dependencies. Many GNU/Linux
distributions already provide this software in their software repositories. If
not, or using Microsoft Windows, please refer to the installation instructions
found on each dependency's homepage. When we know of reliable pre-built Windows packages we will refer to them.

Required Dependencies
---------------------

* Boost  
  Homepage: http://www.boost.org/  
  Modules: chrono, date_time, filesystem, program_options, system, thread   
  Minimum Version: 1.54  
  Windows Binaries: http://sourceforge.net/projects/boost/files/boost-binaries/  

* CMake  
  Homepage: http://www.cmake.org/  
  Minimum Version: 2.8  

* Eigen  
  Homepage: http://eigen.tuxfamily.org/  
  Minimum Version: 3.2.1  

* Git  
  Homepage: http://www.git-scm.com/  
  Minimum Version: 1.7.9  

* OpenSceneGraph  
  Homepage: http://www.openscenegraph.org/  
  Modules: osg, osgViewer, osgText, osgUtil, osgDB, osgGA, osgAnimation  
  Minimum Version: 3.2.0  
  
Optional Dependencies
---------------------

* Doxygen  
  Homepage: http://doxygen.org/  
  Minimum Version: 1.8  

* FreeGlut  
  Homepage: http://freeglut.sourceforge.net/  
  Minimum Version: 2.6.0  

* Google Mock  
  Homepage: https://code.google.com/p/googlemock/  
  Minimum Version: 1.7.0  

* MathJax  
  Homepage: http://www.mathjax.org/  
  Minimum Version: 2.4  

