from svgpathtools import svg2paths2

#Get the x & y coordinates associated with a given timestep
def get_xy(xy_coords, timestep):
    #Check timestep within bounds
    if (timestep >= len(xy_coords)):
        print("Timestep out of bounds")
        return None
    xy = xy_coords[timestep]
    return xy[0], xy[1], xy[2]

#Pre-process SVG file into a list indexed by timestep 
#Each list element contains a tuple of the x, y position and whether the pen should go down or up at the point
def read_SVG(filename, sampling_rate):
    paths, attributes, svg_attributes = svg2paths2(filename)
    dimensions = svg_attributes['viewBox']
    dims = dimensions.split()
    xy_coords = []
    
    #Loop through all segments of each path in SVG
    for path in paths:
        for seg in path:
            #Sample sampling_rate number of points from each segment
            for i in range(0, sampling_rate + 1):
                if (i+1 >= sampling_rate): #End of segment, indicate pen up
                    xy = tuple((seg.point(i/sampling_rate).real, seg.point(i/sampling_rate).imag, 0))
                elif (i == 0): #Beginning of segment, indicate pen down
                    xy = tuple((seg.point(i/sampling_rate).real, seg.point(i/sampling_rate).imag, 1))   
                else: #Otherwise, keep pen down
                    xy = tuple((seg.point(i/sampling_rate).real, seg.point(i/sampling_rate).imag, 1))
                xy_coords.append(xy)
    return xy_coords


#Sample code    
#xy_coords = read_SVG('Examples/SCB.svg', 10)
#x,y,down = get_xy(xy_coords, len(xy_coords) - 12)
#print(x, y, down)

