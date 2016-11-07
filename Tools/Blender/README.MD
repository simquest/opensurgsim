OpenSurgSim Blender Add-ons

Requires MeshPy python library to use the exporter

=== OpenSurgSim Blender Tool Installation ===
1) Open Blender
2) Go to File > User Preferences (Ctrl Alt U)
3) Navigate to the 'Add-ons' tab
4) Select 'Install from File...'
5) Choose the .py file for the add-on
6) This adds it to add-on list, activate it by checking the box on the right hand side

Alternatively you can place the files into Blender's addon folder in your user directory.

Example for Linux and Blender 2.69:

```/home/$user/.blender/2.69/scripts/addons/OpenSurgSim```

Example for Windows and Blender 2.69:

```C:\Users\%username%\AppData\Roaming\Blender Foundation\Blender\2.69\scripts\addons\OpenSurgSim```

You'll still need to activate the addon in Blender

Note: In both scenarios you can find the addon in Blender by searching for 'OpenSurgSim'

=== MeshPy Installation ===
On Linux:
1) Make sure pip is installed (usually installed with Python)
2) Run `pip install meshpy`

On Windows:
Blender comes with its own python installation which gives you the option to install pip on there and install meshpy that way or the following way that worked for me:

1) Download and install Python 3.5 for Windows and add it to the path
2) Download the meshpy wheel for python 3.5 from http://www.lfd.uci.edu/~gohlke/pythonlibs/#meshpy (at the moment that is the unstable one labeled with cp3.5)
3) Install MeshPy via pip with `pip install <filename>`
4) Remove the python folder from your Blender installation so it falls back to using your installed Python and libraries
