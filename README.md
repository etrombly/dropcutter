# dropcutter

CAM software for CNC milling. This is mostly designed for 3D relief carving, but will cut any stl. Supports REST milling. Example usage:

    # cut test.stl with a 5mm endmill and 1mm stepdown
    dropcutter -i test.stl -o test.nc -t endmill -d 5 -s 1
    
    # rest mill test.stl with a 1mm ball mill and 20% stepover
    dropcutter -i test.stl -o test2.nc -t ball -d 1 -s 1 --stepover 20 --restmap rest.map
