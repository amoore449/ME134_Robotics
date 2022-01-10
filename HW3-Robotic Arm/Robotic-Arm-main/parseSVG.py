from svgpathtools import svg2paths2
from get_xy import get_xy, read_SVG
import turtle, time

#Draw processed SVG file with Turtle to simulate
def simulate(filename, sampling_rate):
    paths, attributes, svg_attributes = svg2paths2(filename)

    #Initialize and setup turtle
    s = turtle.Screen()
    t = turtle.Turtle()
    s.reset()
    dimensions = svg_attributes['viewBox']
    dims = dimensions.split()
    s.setworldcoordinates(int(dims[0]), int(dims[1]), int(dims[2]), int(dims[3]))
    t.penup()

    #Process SVG file and draw from data
    xy_coords = read_SVG(filename, 10)
    for i in range(0, len(xy_coords)):
        x,y,down = get_xy(xy_coords, i)
        t.goto(x,y)
        if (down):
            t.pendown()
        else:
            t.penup()

#Run simulation
simulate('Examples/Tamil.svg', 10)

