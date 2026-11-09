"""Browser proof: compiled shaders, stereo parallax and pose response."""
from pathlib import Path
import functools
import http.server
import json
import threading
from PIL import Image,ImageChops,ImageStat
from playwright.sync_api import sync_playwright
from optics import trace_from_eye
from spec import Spec

root=Path(__file__).resolve().parent
out=Path("build/vr_headset/renders");out.mkdir(parents=True,exist_ok=True)
handler=functools.partial(http.server.SimpleHTTPRequestHandler,directory=str(root/"viewer"))
server=http.server.ThreadingHTTPServer(("127.0.0.1",0),handler)
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
errors=[]
try:
    with sync_playwright() as p:
        browser=p.chromium.launch(args=["--use-gl=angle","--use-angle=swiftshader",
                                       "--enable-unsafe-swiftshader"])
        page=browser.new_page(viewport={"width":1440,"height":664},device_scale_factor=1)
        page.on("pageerror",lambda e:errors.append(str(e)))
        page.goto(f"http://127.0.0.1:{server.server_port}")
        page.wait_for_function("window.visor && window.visor.stats().frames>3")
        page.click("#inspect")
        page.screenshot(path=str(out/"visor_stereo_demo.png"))
        stats=page.evaluate("window.visor.stats()")
        assert stats["error"]==0 and not errors,(stats,errors)
        samples=page.evaluate("window.visor.opticalSamples()")
        for angle,x in samples:
            assert abs(x-trace_from_eye(Spec(),angle)["screen_x_mm"])<1e-7
        im=Image.open(out/"visor_stereo_demo.png").convert("RGB")
        left=im.crop((0,50,720,614));right=im.crop((720,50,1440,614))
        difference=sum(ImageStat.Stat(ImageChops.difference(left,right)).mean)
        assert difference>3,difference
        baseline=page.evaluate("window.visor.stats().frames")
        page.evaluate("window.visor.setPose(.3,.1)")
        page.wait_for_function("window.visor.stats().frames>"+str(baseline+3))
        page.screenshot(path=str(out/"visor_stereo_turned.png"))
        changed=sum(ImageStat.Stat(ImageChops.difference(
            im,Image.open(out/"visor_stereo_turned.png").convert("RGB"))).mean)
        assert changed>2,changed
        report={"passed":True,"browser":"Chromium software WebGL2",
                "shader_errors":errors,"stats":stats,"stereo_pixel_difference":difference,
                "pose_pixel_difference":changed,"python_js_ray_agreement":True,
                "phone_sensor_tested":False,"physical_lens_calibration_tested":False}
        (out.parent/"viewer_verification.json").write_text(json.dumps(report,indent=2)+"\n")
        browser.close()
finally:
    server.shutdown()
