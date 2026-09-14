"""Non-neural geometry-guided filtering of the Monte Carlo render.

Reads the renderer's linear-radiance PFM and primary normal / depth / material /
variance buffers. The filter never changes visibility or creates geometry.
Both unfiltered and filtered PNGs are written for inspection.
"""
import os
os.environ.setdefault('NUMBA_NUM_THREADS','4')
import argparse
from pathlib import Path
import numpy as np
from PIL import Image
from numba import njit,prange

@njit(parallel=True,cache=True)
def atrous(im,guide,variance,step,iteration):
    H,W,C=im.shape
    out=np.empty_like(im);vo=np.empty_like(variance)
    kernel=np.array([1.,4.,6.,4.,1.],np.float32)
    for y in prange(H):
        for x in range(W):
            lum=.2126*im[y,x,0]+.7152*im[y,x,1]+.0722*im[y,x,2]
            centerDepth=guide[y,x,6]
            # Robust linear depth prediction keeps slanted metal faces smooth.
            gx=(guide[y,min(x+1,W-1),6]-guide[y,max(x-1,0),6])*.5
            gy=(guide[min(y+1,H-1),x,6]-guide[max(y-1,0),x,6])*.5
            # Clamping prevents depth jumps at a silhouette from permitting
            # a wide filter across that same silhouette.
            gx=max(-1.5,min(1.5,gx));gy=max(-1.5,min(1.5,gy))
            total=0.;value=np.zeros(3,np.float32);vv=0.
            for ky in range(-2,3):
                yy=y+ky*step
                if yy<0 or yy>=H:continue
                for kx in range(-2,3):
                    xx=x+kx*step
                    if xx<0 or xx>=W:continue
                    if guide[y,x,8]!=guide[yy,xx,8]:continue
                    nd=(guide[y,x,0]*guide[yy,xx,0]+guide[y,x,1]*guide[yy,xx,1]+guide[y,x,2]*guide[yy,xx,2])
                    if guide[y,x,8]<0:nd=1.
                    nw=max(nd,0.)**48
                    dz=abs(guide[yy,xx,6]-centerDepth-gx*kx*step-gy*ky*step)
                    dw=np.exp(-dz/(.08+.15*step)) if centerDepth>0 else 1.
                    if guide[y,x,8]==8:dw=1.
                    qlum=.2126*im[yy,xx,0]+.7152*im[yy,xx,1]+.0722*im[yy,xx,2]
                    sigma=variance[y,x]+variance[yy,xx]+.000005
                    # Broader early filtering estimates smooth reflected light;
                    # later passes use propagated variance and preserve edges.
                    cw=np.exp(-(lum-qlum)**2/(sigma*9.+.00003))
                    w=kernel[ky+2]*kernel[kx+2]*nw*dw*cw
                    total+=w
                    for c in range(3):value[c]+=im[yy,xx,c]*w
                    vv+=variance[yy,xx]*w*w
            if total>1e-10:
                for c in range(3):out[y,x,c]=value[c]/total
                vo[y,x]=vv/(total*total)
            else:
                out[y,x]=im[y,x];vo[y,x]=variance[y,x]
    return out,vo

def tonemap(x,exposure=1.):
    x=np.maximum(x*exposure,0.)
    x=np.clip(x*(2.51*x+.03)/(x*(2.43*x+.59)+.14),0.,1.)
    x=np.where(x<=.0031308,12.92*x,1.055*x**(1/2.4)-.055)
    return np.uint8(np.clip(x*255+.5,0,255))

def read_pfm(path):
    with open(path,'rb') as f:
        if f.readline().strip()!=b'PF':raise ValueError('Expected RGB PFM')
        w,h=map(int,f.readline().split());scale=float(f.readline())
        a=np.fromfile(f,dtype='<f4' if scale<0 else '>f4').reshape(h,w,3)[::-1].copy()
    return a

def main():
    p=argparse.ArgumentParser();p.add_argument('ppm');p.add_argument('--output');p.add_argument('--passes',type=int,default=3);p.add_argument('--exposure',type=float,default=1.)
    args=p.parse_args();src=Path(args.ppm);dest=Path(args.output) if args.output else src.with_suffix('.png')
    im=read_pfm(str(src)+'.pfm')
    with open(str(src)+'.guides','rb') as f:
        w,h=np.fromfile(f,dtype='<u4',count=2);guide=np.fromfile(f,dtype='<f4').reshape(h,w,9)
    Image.fromarray(tonemap(im,args.exposure)).save(dest.with_stem(dest.stem+'_raw'))
    v=guide[:,:,7].copy()
    for i in range(args.passes):
        im,v=atrous(im,guide,v,2**i,i)
    Image.fromarray(tonemap(im,args.exposure)).save(dest)
    print(dest,flush=True)
if __name__=='__main__':main()
