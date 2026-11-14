"""Original fictional ceremonial weapon, constructed with CYBR GEO.
Run: python -m examples.weapon_asset --out build/weapon
Requires cybrgeo, numpy, matplotlib, imageio; Mitsuba 3 optional.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import cybrgeo as geo

PARTS = []
MATERIALS = {
    "steel": (.65, .72, .79),
    "bronze": (.54, .32, .12),
    "titanium": (.10, .13, .17),
    "leather": (.07, .045, .035),
}

def add(name, mesh, material):
    PARTS.append((name, mesh, material))
    return mesh

def ellipse(cx, cy, rx, ry, n=64):
    t = np.linspace(0, 2*np.pi, n, endpoint=False)
    return np.column_stack((cx + rx*np.cos(t), cy + ry*np.sin(t)))

def extrude(name, profile, z, height, material):
    return add(name, geo.extrude(profile, height, z=z), material)

def loft(name, sections, material):
    return add(name, geo.loft(np.asarray(sections, float), caps=True), material)

def build():
    # Blade: diamond cross-sections along a shared spine.
    stations = [(.25,.13,.052),(.38,.17,.054),(.72,.205,.050),
                (1.12,.198,.046),(1.48,.165,.038),(1.78,.115,.027),
                (2.05,.050,.014),(2.14,.0015,.0015)]
    sections = [[[-w,y,0],[0,y,t],[w,y,0],[0,y,-t]]
                for y,w,t in stations]
    loft("faceted blade", sections, "steel")
    # Raised longitudinal inlays on both faces.
    for sign in (-1,1):
        sections = []
        for y,w,t in [(.42,.014,.053),(.8,.014,.050),
                      (1.2,.012,.045),(1.58,.009,.034),(1.85,.004,.022)]:
            sections.append([[-w,y,sign*(t+.001)],[w,y,sign*(t+.001)],
                             [w,y,sign*(t+.0025)],[-w,y,sign*(t+.0025)]])
        loft("blade inlay %s" % sign, sections, "bronze")
    # Two sculptural crossguard wings.
    for sign in (-1,1):
        sections = []
        for x,span,depth,y in [(.07,.075,.080,.27),(.20,.092,.077,.27),
                               (.35,.080,.070,.29),(.50,.058,.058,.33),
                               (.63,.027,.036,.39)]:
            x *= sign
            sections.append([[x,y-span,-depth],[x,y+span,-depth],
                             [x,y+span,depth],[x,y-span,depth]])
        loft("guard wing %s" % sign, sections, "titanium")
    extrude("guard boss",ellipse(0,.28,.14,.105),-.09,.18,"bronze")
    extrude("grip",ellipse(0,-.12,.070,.30),-.052,.104,"leather")
    for i in range(13):
        extrude("grip band %02d"%i,ellipse(0,-.39+i*.044,.077,.011),
                -.063,.126,"titanium")
    extrude("pommel shoulder",ellipse(0,-.48,.105,.065),-.083,.166,"bronze")
    extrude("pommel",ellipse(0,-.565,.082,.085),-.085,.17,"titanium")
    extrude("pommel inset",ellipse(0,-.575,.032,.038),-.092,.184,"bronze")
    return PARTS

def export(out):
    manifest = []
    for i,(name,mesh,material) in enumerate(PARTS):
        filename = "part_%03d.obj" % i
        geo.export_obj(mesh,out/filename)
        manifest.append(dict(name=name,material=material,file=filename,
                             vertices=len(mesh.vertices),faces=len(mesh.faces),
                             topology=mesh.report()))
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2,default=str))
    return manifest

def mitsuba_scene(out, manifest, clay=False, spp=256):
    # Mitsuba 3 XML, with geometry loaded directly from CYBR GEO OBJ exports.
    import xml.etree.ElementTree as ET
    root = ET.Element("scene",version="3.0.0")
    ET.SubElement(root,"integrator",type="path",max_depth="12")
    sensor = ET.SubElement(root,"sensor",type="perspective")
    ET.SubElement(sensor,"float",name="fov",value="34")
    transform = ET.SubElement(sensor,"transform",name="to_world")
    ET.SubElement(transform,"lookat",origin="3.3,1.1,5.2",
                  target="0,0.72,0",up="0,1,0")
    sampler = ET.SubElement(sensor,"sampler",type="independent")
    ET.SubElement(sampler,"integer",name="sample_count",value=str(spp))
    film = ET.SubElement(sensor,"film",type="hdrfilm")
    ET.SubElement(film,"integer",name="width",value="1100")
    ET.SubElement(film,"integer",name="height",value="1100")
    for part in manifest:
        shape = ET.SubElement(root,"shape",type="obj")
        ET.SubElement(shape,"string",name="filename",value=part["file"])
        bsdf = ET.SubElement(shape,"bsdf",type="diffuse" if clay else "roughplastic")
        color = (.65,.65,.65) if clay else MATERIALS[part["material"]]
        ET.SubElement(bsdf,"rgb",name="reflectance" if clay else "diffuse_reflectance",
                      value=",".join(map(str,color)))
        if not clay:
            ET.SubElement(bsdf,"float",name="alpha",value=".18")
    for x,y,z,power in [(2,5,4,70),(-4,2,1,50),(0,1,-5,90)]:
        emitter = ET.SubElement(root,"emitter",type="point")
        transform = ET.SubElement(emitter,"transform",name="to_world")
        ET.SubElement(transform,"translate",x=str(x),y=str(y),z=str(z))
        ET.SubElement(emitter,"rgb",name="intensity",
                      value=",".join([str(power)]*3))
    ET.indent(root)
    path = out/("mitsuba_clay.xml" if clay else "mitsuba_beauty.xml")
    ET.ElementTree(root).write(path,encoding="unicode",xml_declaration=True)
    return path

def inspect(out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    for mode in ("clay","wireframe"):
        fig=plt.figure(figsize=(12,12),facecolor="#171a1d")
        for index,(elev,azim) in enumerate([(14,-70),(0,-90),(70,-90),(15,-30)]):
            ax=fig.add_subplot(2,2,index+1,projection="3d",facecolor="#171a1d")
            for name,mesh,material in PARTS:
                vertices=np.asarray(mesh.vertices)
                faces=np.asarray(mesh.faces)
                poly=Poly3DCollection(vertices[faces],
                    linewidths=.12 if mode=="wireframe" else 0,
                    edgecolors="#8aa1b1" if mode=="wireframe" else "none",
                    facecolors=(.12,.18,.23,.18) if mode=="wireframe"
                               else (.68,.68,.67,1))
                ax.add_collection3d(poly)
            ax.set_xlim(-1,1);ax.set_ylim(-.8,2.3);ax.set_zlim(-1,1)
            ax.set_box_aspect((2,3.1,2));ax.view_init(elev=elev,azim=azim)
            ax.set_axis_off()
            ax.set_title(["THREE-QUARTER","FRONT","TOP","REAR"][index],color="white")
        fig.savefig(out/(mode+"_contact_sheet.png"),dpi=140,
                    facecolor=fig.get_facecolor())
        plt.close(fig)

def construction_video(out):
    # Cumulative authored-part replay, not an operation-level Boolean replay.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import imageio.v2 as imageio
    import inspect as pyinspect
    lines=pyinspect.getsource(build).splitlines()
    frames=[]
    for count in range(1,len(PARTS)+1):
        fig=plt.figure(figsize=(12,7),facecolor="#10151b")
        left=fig.add_axes([.025,.05,.44,.9]);left.set_axis_off()
        left.text(0,.99,"CYBR GEO / BUILD SOURCE",color="white",
                  va="top",fontsize=13,fontfamily="monospace")
        left.text(0,.92,"\n".join(lines[:33]),color="#9fb0c0",
                  va="top",fontsize=7,fontfamily="monospace")
        left.text(0,.04,"CURRENT PART: "+PARTS[count-1][0],color="white",
                  fontsize=11,fontfamily="monospace")
        ax=fig.add_axes([.48,.04,.50,.92],projection="3d",facecolor="#10151b")
        for name,mesh,material in PARTS[:count]:
            v=np.asarray(mesh.vertices);f=np.asarray(mesh.faces)
            ax.add_collection3d(Poly3DCollection(v[f],
                                facecolors=MATERIALS[material],edgecolors="none"))
        ax.set_xlim(-1,1);ax.set_ylim(-.8,2.3);ax.set_zlim(-1,1)
        ax.set_box_aspect((2,3.1,2));ax.view_init(elev=12,azim=-65)
        ax.set_axis_off()
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba())[...,:3].copy())
        plt.close(fig)
    imageio.mimsave(out/"construction_replay.mp4",frames,fps=2,
                    macro_block_size=16)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--out",default="build/weapon")
    parser.add_argument("--render",action="store_true")
    parser.add_argument("--skip-video",action="store_true")
    args=parser.parse_args()
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=True)
    build()
    manifest=export(out)
    for clay in (False,True):
        path=mitsuba_scene(out,manifest,clay=clay)
        if args.render:
            import mitsuba as mi
            mi.set_variant("scalar_rgb")
            image=mi.render(mi.load_file(str(path)))
            name="clay" if clay else "beauty"
            mi.util.write_bitmap(str(out/("mitsuba_"+name+".exr")),image)
            mi.util.write_bitmap(str(out/("mitsuba_"+name+".png")),image)
    inspect(out)
    if not args.skip_video:
        construction_video(out)
    print("Exported",len(manifest),"components to",out)

if __name__=="__main__":
    main()
