Scripts in here might have a very specific use case or application, or there might be a need to preprocess or postprocess to utilize them these could be refactored into something more general. 

merge.py:
	Merge can take a reference list of vertices from a .ply file and add texture coordinates from a .obj file without changing the order of the vertices in the .ply description. It does not implement a full .ply/.obj parser. See the documentation in the file for more information