"""Join disjoint V9 time segments and verify identical decoded frame hashes."""
from pathlib import Path
import hashlib
import json
import subprocess

root=Path("build/vr_headset/renders")
segments=[root/f"visor_ipd_segment_{i}.mp4" for i in range(4)]
reports=[json.loads(p.with_suffix(".json").read_text()) for p in segments]
for i,r in enumerate(reports):
    assert r["frames"]==24 and r["time_offset_seconds"]==i
    assert r["render_profile"]=="v9" and not r["generated_imagery"]
    assert not r["frame_interpolation"]
    assert r["spp_per_frame"]==64 and r["resolution"]==[800,600]
    assert r["max_depth"]==12 and r["fps"]==24
listing=root/"segments.txt"
listing.write_text("".join("file '"+str(p.resolve())+"'\n" for p in segments))
video=root/"visor_ipd_motion.mp4"
subprocess.run(["ffmpeg","-y","-v","error","-f","concat","-safe","0","-i",
                str(listing),"-c","copy","-movflags","+faststart",str(video)],check=True)
def hashes(path):
    p=subprocess.run(["ffmpeg","-v","error","-xerror","-i",str(path),
                      "-map","0:v:0","-f","framemd5","-"],
                     check=True,capture_output=True,text=True)
    return [line.rsplit(",",1)[-1].strip() for line in p.stdout.splitlines()
            if line and not line.startswith("#")]
original=[h for p in segments for h in hashes(p)]
joined=hashes(video)
assert len(original)==len(joined)==96
assert original==joined,"Concatenation changed or reordered decoded frames"
probe=json.loads(subprocess.run(["ffprobe","-v","error","-count_frames",
    "-show_entries","stream=width,height,nb_read_frames,r_frame_rate:format=duration",
    "-of","json",str(video)],capture_output=True,text=True,check=True).stdout)
assert int(probe["streams"][0]["nb_read_frames"])==96
assert abs(float(probe["format"]["duration"])-4)<.05
report={"model":"cybr_visor_m1","render_profile":"v9","frames":96,"fps":24,
        "duration":4,"resolution":[800,600],"spp_per_frame":64,"max_depth":12,
        "generated_imagery":False,"frame_interpolation":False,
        "segments":[p.name for p in segments],"segment_reports":reports,
        "concatenation":"stream copy; every decoded frame hash equals its source frame",
        "decoded_frame_hashes":joined,"probe":probe,
        "sha256":hashlib.sha256(video.read_bytes()).hexdigest(),
        "physical_headset_tested":False}
video.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n")
subprocess.run(["ffmpeg","-y","-v","error","-i",str(video),"-filter_complex",
 "fps=12,scale=600:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=sierra2_4a",
 "-loop","0",str(root/"visor_ipd_motion.gif")],check=True)
# The delivered motion contact sheet uses four actual decoded frames.
for index in (0,24,48,72):
    subprocess.run(["ffmpeg","-y","-v","error","-i",str(video),"-vf",
                    f"select=eq(n\\,{index})","-frames:v","1",
                    str(root/f"visor_motion_frame_{index:03}.png")],check=True)
print("VERIFIED 96 unchanged decoded frames, 4 seconds at 24 fps",flush=True)
