from klampt import vis
from scipy.spatial import ConvexHull
import draw_hull
from OpenGL.GL import *

# h = ConvexHull([[0,0,0],[0,0,1],[0,1,0],[1,0,0]])
h = ConvexHull([[4.092341, -0.159296, -0.0], [5.0, -0.201041, 1.405738], [4.306438, -0.165628, -0.0], [4.096664, -0.021364, -0.0], [5.0, 0.18086, 0.696054], [5.0, 0.238023, 0.489633], [5.0, 0.104964, 0.453037], [4.313766, -0.029114, -0.0]])
hrender = draw_hull.PrettyHullRenderer(h)
vis.add("blah",h)
#vis.setDrawFunc("blah",lambda h:hrender.render())
def my_draw_hull(h):
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    # glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,0.5])
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1.0,0.25,0.5,0.25])
    draw_hull.draw_hull(h)
vis.setDrawFunc("blah",my_draw_hull)

vis.run()
