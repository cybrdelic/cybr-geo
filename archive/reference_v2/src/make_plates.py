"""Compose comparison / multi-angle sheets from the actual rendered images."""
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[1]
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def font(n,bold=False):return ImageFont.truetype(BOLD if bold else FONT,n)
BG=(19,24,27);INK=(224,230,232);MUTED=(141,156,164);LINE=(65,80,87)
def contain(im,box):
    im=im.copy();im.thumbnail((box[2],box[3]),Image.Resampling.LANCZOS)
    return im,(int(box[0]+(box[2]-im.width)/2),int(box[1]+(box[3]-im.height)/2))
def cropped(name):
    im=Image.open(R/'renders'/f'{name}.png').convert('RGB')
    path=R/'renders'/f'{name}.ppm.guides'
    if path.exists():
        with open(path,'rb') as f:
            w,h=np.fromfile(f,'<u4',2);g=np.fromfile(f,'<f4').reshape(h,w,9)
        y,x=np.where((g[:,:,8]>=0)&(g[:,:,8]<8))
        if len(x):
            pad=max(24,int((x.max()-x.min())*.065))
            im=im.crop((max(0,int(x.min())-pad),max(0,int(y.min())-pad),min(im.width,int(x.max())+pad),min(im.height,int(y.max())+pad)))
    return im

def multi():
    out=Image.new('RGB',(2520,1610),BG);d=ImageDraw.Draw(out)
    d.text((55,38),'TORSEN-X / REBUILT GEOMETRY',font=font(37),fill=INK)
    d.text((57,95),'Five views of one procedural model · offline path-traced stills',font=font(22),fill=MUTED)
    d.line((55,140,2465,140),fill=LINE,width=1)
    im,xy=contain(cropped('hero'),(35,210,1175,1200));out.paste(im,xy)
    d.text((58,165),'01 / ASSEMBLED HERO',font=font(22),fill=INK)
    configs=[('rear','02 / REAR',1260,165),('front','03 / FRONT — ORTHOGRAPHIC',1890,165),('side','04 / SIDE — ORTHOGRAPHIC',1260,830),('internals','05 / OUTER PARTS REMOVED',1890,830)]
    for name,label,x,y in configs:
        d.text((x,y),label,font=font(18),fill=INK)
        im,xy=contain(cropped(name),(x-5,y+47,590,545));out.paste(im,xy)
    d.line((55,1490,2465,1490),fill=LINE,width=1)
    d.text((55,1515),'Actual CAD / mesh surfaces. No generated imagery is used in these five rendered views.',font=font(22),fill=INK)
    d.text((55,1555),'Visual reconstruction of the principal reference image; not a mechanically validated differential.',font=font(20),fill=MUTED)
    out.save(R/'renders/multiangle.png')

def compare():
    out=Image.new('RGB',(2400,1320),BG);d=ImageDraw.Draw(out)
    d.text((45,32),'REFERENCE / GEOMETRY REBUILD',font=font(34),fill=INK)
    d.text((45,86),'The large hero image is the modeling target.',font=font(22),fill=MUTED)
    panels=[(Image.open(R/'reference_hero_crop.png').convert('RGB'),'ORIGINAL GENERATED REFERENCE',35),
            (cropped('hero'),'NEW GEOMETRY — OFFLINE RENDER',1235)]
    for im,label,x in panels:
        d.text((x+10,145),label,font=font(23),fill=INK)
        im,xy=contain(im,(x,197,1125,975));out.paste(im,xy)
    d.line((1200,145,1200,1202),fill=LINE,width=1)
    d.line((45,1212,2355,1212),fill=LINE,width=1)
    d.text((45,1237),'The right-hand image is rendered entirely from the included mesh. It is a visual reconstruction, not an exact CAD recovery.',font=font(20),fill=MUTED)
    out.save(R/'renders/reference_comparison.png')

if __name__=='__main__':multi();compare()
