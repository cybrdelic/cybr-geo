import numpy as np
from cybrgeo import sweep, sweep_shell, bezier, sample_curve, loft
scene = []
a = np.linspace(0, 2*np.pi, 16, endpoint=False)
circle = np.column_stack((np.cos(a), np.sin(a)))
rear, front = np.array([0.,0.,.34]), np.array([1.06,0.,.34])
bb, seat = np.array([.41,0.,.29]), np.array([.28,0.,.82])
head_low, head_top = np.array([.91,0.,.68]), np.array([.87,0.,.84])
frame_edges = [(bb,seat,.017),(seat,head_top,.015),(bb,head_low,.022),(head_low,head_top,.023)]
for start, end, radius in frame_edges:
    path = np.array([start,end])
    part = sweep_shell(circle*radius,circle*(radius-.002),path)
    scene.append((part,(48,122,124),'frame tube'))
for side in (-1,1):
    dropout = rear + [0,side*.065,0]
    for joint in (bb,seat):
        path = sample_curve(lambda t: bezier([joint,joint+[ -.1,side*.04,0],dropout+[.10,0,0],dropout],t),count=18)
        part = sweep(circle*.009,path)
        scene.append((part,(48,122,124),'rear stay'))
    path = sample_curve(lambda t: bezier([head_low,head_low+[.03,side*.05,-.12],front+[-.06,side*.055,.06],front+[0,side*.055,0]],t),count=24)
    part = sweep(circle*.013,path)
    scene.append((part,(48,122,124),'curved fork blade'))
for hub in (rear,front):
    theta = np.linspace(0,2*np.pi,121)
    path = hub + np.column_stack((.323*np.cos(theta),np.zeros(len(theta)),.323*np.sin(theta)))
    part = sweep(circle*.017,path,caps=False)
    scene.append((part,(45,49,52),'tire'))
    path = hub + np.column_stack((.303*np.cos(theta),np.zeros(len(theta)),.303*np.sin(theta)))
    part = sweep(circle*.012,path,caps=False)
    scene.append((part,(170,178,182),'rim'))
    path = np.array([hub+[0,-.065,0],hub+[0,.065,0]])
    part = sweep(circle*.022,path)
    scene.append((part,(155,166,173),'hub barrel'))
    for spoke in range(32):
        phase = spoke*2*np.pi/32
        side = (-1)**spoke
        start = hub+[.025*np.cos(phase+.3),side*.045,.025*np.sin(phase+.3)]
        end = hub+[.303*np.cos(phase),0,.303*np.sin(phase)]
        path = np.array([start,end])
        part = sweep(circle*.0013,path)
        scene.append((part,(149,158,164),'cross-laced spoke'))
path = np.array([bb+[0,-.08,0],bb+[0,.08,0]])
part = sweep(circle*.026,path)
scene.append((part,(145,155,161),'bottom bracket'))
for center,radius in ((bb,.096),(rear,.040)):
    theta = np.linspace(0,2*np.pi,193)
    path = center+[0,-.078,0]+np.column_stack((radius*np.cos(theta),np.zeros(len(theta)),radius*np.sin(theta)))
    part = sweep(circle*.005,path,caps=False)
    scene.append((part,(173,179,182),'chainring'))
    for i in range(5):
        phase = i*2*np.pi/5
        path = np.array([center+[0,-.078,0],center+[radius*np.cos(phase),-.078,radius*np.sin(phase)]])
        part = sweep(circle*.004,path)
        scene.append((part,(151,160,167),'chainring spider'))
chain_top = np.array([rear+[0,-.078,.04],bb+[0,-.078,.096]])
chain_bottom = np.array([rear+[0,-.078,-.04],bb+[0,-.078,-.096]])
for path in (chain_top,chain_bottom):
    part = sweep(circle*.003,path)
    scene.append((part,(94,103,109),'chain straight run'))
for side in (-1,1):
    pedal = bb + [side*.12,side*.095,side*.12]
    path = np.array([bb+[0,side*.095,0],pedal])
    part = sweep(circle*.012,path,scale=[1,.75])
    scene.append((part,(145,154,163),'crank arm'))
    path = np.array([pedal,pedal+[0,side*.07,0]])
    part = sweep(np.array([[-.04,-.012],[.04,-.012],[.04,.012],[-.04,.012]]),path)
    scene.append((part,(61,68,74),'pedal'))
path = np.array([seat,seat+[-.028,0,.11]])
part = sweep(circle*.013,path)
scene.append((part,(150,161,171),'seatpost'))
saddle_outline = np.array([[-.13,-.064],[-.06,-.072],[.13,-.018],[.16,0],[.13,.018],[-.06,.072],[-.13,.064]])
sections = [np.column_stack((saddle_outline[:,0]+.265,saddle_outline[:,1],np.full(7,z))) for z in (.925,.946)]
part = loft(sections)
scene.append((part,(69,50,43),'saddle'))
path = np.array([head_top,head_top+[-.02,0,.10],head_top+[.07,0,.10]])
part = sweep(circle*.012,path)
scene.append((part,(155,164,171),'stem'))
for side in (-1,1):
    controls = np.array([[.94,0,.94],[.94,side*.13,.94],[.90,side*.23,.97],[.83,side*.25,.97]])
    path = sample_curve(lambda t: bezier(controls,t),count=25)
    part = sweep(circle*.011,path)
    scene.append((part,(153,162,170),'handlebar'))
    part = sweep(circle*.016,path[-9:])
    scene.append((part,(60,64,68),'grip'))
    controls = np.array([[.86,side*.24,.97],[1.02,side*.16,.86],[.75,side*.08,.77],[.94,side*.02,.59]])
    path = sample_curve(lambda t: bezier(controls,t),count=32)
    part = sweep(circle*.002,path)
    scene.append((part,(58,65,70),'brake cable'))
for side in (-1,1):
    path = np.array([[.94,side*.02,.59],[.94,side*.044,.64],[.965,side*.025,.625]])
    part = sweep(circle*.006,path)
    scene.append((part,(157,165,171),'front brake arm'))
    path = np.array([[.95,side*.021,.625],[.98,side*.021,.625]])
    part = sweep(circle*.007,path)
    scene.append((part,(51,56,61),'rim brake pad'))
