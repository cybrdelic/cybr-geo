"""Actual CYBR GEO V9 renders; image generation is never used."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recipe import pose
from mechanism_lab.core import load_cache
from mechanism_lab.media import Shot,make_gif
from mechanism_lab.v9_dispatch import render_v9,render_v9_video
from PIL import Image,ImageStat

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--out",type=Path,default=Path("build/vr_headset"))
    ap.add_argument("--views",nargs="+",default=["hero","eyes","exploded"])
    ap.add_argument("--spp",type=int,default=256)
    ap.add_argument("--video",action="store_true")
    ap.add_argument("--segment-index",type=int,choices=range(4))
    args=ap.parse_args()
    a=load_cache(args.out/"cache");a.motion_function=pose
    folder=args.out/"renders";folder.mkdir(exist_ok=True)
    for view in ([] if args.segment_index is not None else args.views):
        image=folder/f"visor_{view}.png"
        report=render_v9(a,image,view_name=view,size=(1100,825),spp=args.spp,
                         depth=14,intent="inspection")
        with Image.open(image) as im:
            assert im.size==(1100,825)
            assert max(ImageStat.Stat(im.convert("RGB")).stddev)>12
        assert report["generated_imagery"] is False
        assert report["render_profile"]=="v9"
        report["sha256"]=hashlib.sha256(image.read_bytes()).hexdigest()
        report["physical_headset_tested"]=False
        image.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n")
    if args.video or args.segment_index is not None:
        segment=args.segment_index
        duration=1 if segment is not None else 4
        if segment is not None:
            a.motion_function=lambda part,t=0.,explode=0.:pose(part,t+segment,explode)
        path=folder/(f"visor_ipd_segment_{segment}.mp4" if segment is not None else "visor_ipd_motion.mp4")
        # Every encoded frame is independently path traced. This is a 4-second
        # geometric proof at a stated preview budget, not a 256-spp final film.
        report=render_v9_video(a,path,[Shot("optics",duration,"motion")],
            size=(800,600),fps=24,spp=64,depth=12,intent="inspection")
        assert report["frames"]==24*duration and report["frame_interpolation"] is False
        report["time_offset_seconds"]=segment or 0
        path.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n")
        if segment is None:
            make_gif(path,folder/"visor_ipd_motion.gif",width=600,fps=12,seconds=4)

if __name__=="__main__":
    main()
