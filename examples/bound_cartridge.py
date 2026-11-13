"""Geometry-driven bearing cartridge. Millimetres; not an engineering certification."""
import numpy as np
import cybrgeo as geo

def circle(radius, n=64):
    angle = np.linspace(0, 2*np.pi, n, endpoint=False)
    return np.c_[radius*np.cos(angle), radius*np.sin(angle)]

def tagged(mesh, label, mask):
    return geo.Mesh(mesh.vertices, mesh.faces,
                    {**mesh.attributes, label: geo.Attribute(mask.astype(float), interpolation='nearest')})

def housing(radius, offset):
    holes = [circle(radius)+offset]
    holes += [circle(2.2,24)+[x,y] for x in [-23,23] for y in [-19,19]]
    mesh = geo.extrude(np.array([[-30,-26],[30,-26],[30,26],[-30,26]]), 8, holes=holes)
    p = mesh.vertices
    rim = np.isclose(p[:,2],8) & np.isclose(np.linalg.norm(p[:,:2]-offset,axis=1),radius)
    return tagged(mesh,'bearing-seat',rim)

def rotor(frame):
    radius = frame.fitted_radius(.2)
    mesh = geo.extrude(circle(radius), 31, z=-24)
    return tagged(mesh,'shoulder',np.isclose(mesh.vertices[:,2],0-24))

def align(mesh, frame, angle):
    source = geo.Feature('shoulder').resolve(mesh)
    return geo.place(mesh,source,frame,angle=angle,slide=-24)

def build_graph():
    graph = geo.Graph()
    graph.input('bore-radius',9.)
    graph.input('bore-offset',np.array([0.,0.]))
    graph.input('rotation',0.)
    graph.op('housing',housing,'bore-radius','bore-offset')
    graph.op('seat',geo.Feature('bearing-seat').resolve,'housing')
    graph.op('shaft',rotor,'seat')
    graph.op('placed-shaft',align,'shaft','seat','rotation')
    return graph

def decorated(graph):
    b = graph.build(['housing','seat','placed-shaft'])
    frame = b.outputs['seat']
    scene = [('housing',b.outputs['housing'],(60,106,130)),
             ('shaft',b.outputs['placed-shaft'],(167,182,188))]
    # Concentric race and flange derived from the same measured feature.
    r = frame.radius
    for name,ri,ro,z,h,color in [
        ('bearing race',r+.05,r+3,8,3,(195,153,68)),
        ('shaft collar',r-.2,r+2,12,3,(147,162,168)),
        ('end flange',r-.2,r+8,-18,3,(117,137,148))]:
        m = geo.extrude(circle(ro),h,holes=[circle(ri)],z=z)
        T = np.eye(4); T[:2,3] = frame.origin[:2]
        scene.append((name,geo.transform(m,T),color))
    for x in [-23,23]:
        for y in [-19,19]:
            bolt = geo.extrude(circle(3.6,6),3,z=8)
            T = np.eye(4);T[:3,3] = [x,y,0]
            scene.append(('hex fastener',geo.transform(bolt,T),(140,151,158)))
    return scene

if __name__ == '__main__':
    graph = build_graph()
    scene = decorated(graph)
    graph.set('bore-radius',11.)
    graph.set('bore-offset',np.array([5.,3.]))
    scene = decorated(graph)
