## OpenSurgSim Blender Add-ons ##
There are two addons included; the oss_fem addon allows you to set the selected vertices 
as boundary conditions. The oss_export addon exports the selected object as an FEM of the 
desired dimension. The exporter requires the MeshPy python library to be installed.

### OpenSurgSim Blender Tool Installation ###
1. Open Blender
2. Go to File > User Preferences (Ctrl Alt U.
3. Navigate to the 'Add-ons' tab
4. Select 'Install from File...'
5. Choose the .py file for the add-on
6. This adds it to add-on list, activate it by checking the box on the right hand side

Alternatively you can place the files into Blender's addon folder in your user directory.

Example for Linux and Blender 2.69:

```/home/$user/.blender/2.69/scripts/addons/OpenSurgSim```

Example for Windows and Blender 2.69:

```C:\Users\%username%\AppData\Roaming\Blender Foundation\Blender\2.69\scripts\addons\OpenSurgSim```

You'll still need to activate the addon in Blender

Note: In both scenarios you can find the addon in Blender by searching for 'OpenSurgSim'
in User Preferences > Add-ons

### Using oss_export ###
The exporter adds a new 'OpenSurgSim FEM' option to the 'File > Export' menu. It is important to note that the exporter
is only capable of exporting a single object at a time. Select the object you wish to export as an FEM and then go to
'File > Export > OpenSurgSim FEM (.ply)' which will provide with a save dialog with a few options:
- Export Graphics: This will include faces of the mesh as they are in Blender for use in a graphics or collision
representation, as well as texture coordinates. The texture export is still an experimental feature so mileage may vary.
- Dimension: Specifies how the exporter should treat the mesh when writing out to the file. 
- Mass Density
- Poisson Ratio
- Young Modulus
- MeshPy Options: The arguments (based on tetgen arguments) to pass to MeshPy for FEM 3D tetrahedron generation

### Using oss_fem ###
This add-on works only when you are in edit mode in Blender. When you are in edit mode, select the vertices you
wish to mark as boundary conditions. In the 'Mesh' menu you will find the option 'Set Boundary Condition'. You can also
press the spacebar to bring up the search menu and search for that action. If you wish to see the list of vertex indices
selected, go to the Object tab in the Properties Panel and look under the Custom Properties section

### MeshPy Installation ###
On Linux:
1. Make sure pip is installed (at this point Blender uses Python3 so you'll need the appropriate pip for that version)
2. Run `pip install meshpy` (or on some systems: `pip3 install meshpy`)

On Windows:
Blender comes with its own python installation which gives you the option to install pip on there and install meshpy 
that way or the following way that worked for me:

1. Install pip to the python installation packaged with Blender
2. Install MeshPy via pip with `pip install meshpy`
3. Remove the python folder from your Blender installation so it falls back to using your installed Python and libraries
