"""Qualify job-local Mesa llvmpipe without changing Windows or Python installs.

VTK 9.6.2 points Windows software-rendering users at mesa-dist-win:
https://github.com/Kitware/VTK/blob/v9.6.2/Rendering/OpenGL2/vtkOSOpenGLRenderWindow.cxx
The current distribution documents its WGL DLL pair and GALLIUM_DRIVER here:
https://github.com/pal1000/mesa-dist-win/blob/26.2.4/readme.md
The SHA256 below is the GitHub release asset's published digest, not computed
from a download during CI. No deployment scripts or system drivers are run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import urllib.request


URL = 'https://github.com/pal1000/mesa-dist-win/releases/download/26.2.4/mesa3d-26.2.4-release-msvc.7z'
SHA256 = '351fc8c8b695878ffb3eaa044b3ead08672a48b1a045e3c3e3975811df0f6695'
FILES = ('x64/opengl32.dll', 'x64/libgallium_wgl.dll')


def prepare(directory: Path, evidence: Path, archive: Path | None = None):
    if sys.platform != 'win32' or platform.machine().lower() not in ('amd64', 'x86_64'):
        raise RuntimeError('This software OpenGL package is for Windows x64 only')
    directory = directory.resolve()
    evidence = evidence.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    evidence.mkdir(parents=True, exist_ok=True)
    archive = archive.resolve() if archive else directory / 'mesa.7z'
    if not archive.exists():
        with urllib.request.urlopen(URL, timeout=60) as response, archive.open('wb') as output:
            shutil.copyfileobj(response, output)
    actual_hash = hashlib.sha256(archive.read_bytes()).hexdigest()
    if actual_hash != SHA256:
        raise RuntimeError(f'Mesa archive SHA256 mismatch: {actual_hash}')
    # Windows bsdtar supports 7z. Extract only the documented x64 WGL pair;
    # in particular, never execute the archive's deployment scripts.
    subprocess.run(['tar.exe', '-xf', str(archive), '-C', str(directory), *FILES], check=True)
    libraries = directory / 'x64'
    for name in FILES:
        if not (directory / name).is_file():
            raise RuntimeError(f'Mesa archive is missing {name}')
    bootstrap = directory / 'bootstrap'
    bootstrap.mkdir(exist_ok=True)
    # Load before any VTK/CadQuery import so their OpenGL dependency resolves to
    # this process-local module. Keep the DLL search handle alive for the process.
    # An absolute path and the verified archive avoid relying on PATH DLL lookup.
    (bootstrap / 'sitecustomize.py').write_text(
        'import ctypes, os\n'
        f'_mesa_directory = {str(libraries)!r}\n'
        '_mesa_search = os.add_dll_directory(_mesa_directory)\n'
        '_mesa_gl = ctypes.WinDLL(os.path.join(_mesa_directory, "opengl32.dll"))\n',
        encoding='utf-8')
    settings = {
        'PYTHONPATH': str(bootstrap) + os.pathsep + os.environ.get('PYTHONPATH', ''),
        'GALLIUM_DRIVER': 'llvmpipe',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'VTK_DEFAULT_OPENGL_WINDOW': 'vtkWin32OpenGLRenderWindow',
    }
    environment = {**os.environ, **settings}
    probe = '''import hashlib, json, pathlib, vtk
from PIL import Image, ImageStat
out = pathlib.Path(__import__('sys').argv[1])
sphere = vtk.vtkSphereSource(); sphere.SetThetaResolution(16); sphere.SetPhiResolution(16)
mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
actor = vtk.vtkActor(); actor.SetMapper(mapper)
actor.GetProperty().SetInterpolationToPBR(); actor.GetProperty().SetColor(.2, .7, .4)
actor.GetProperty().SetMetallic(.3); actor.GetProperty().SetRoughness(.4)
renderer = vtk.vtkRenderer(); renderer.SetBackground(.04, .05, .07); renderer.AddActor(actor)
window = vtk.vtkRenderWindow(); window.SetOffScreenRendering(1)
window.SetSize(64, 64); window.AddRenderer(renderer); renderer.ResetCamera(); window.Render()
capabilities = window.ReportCapabilities()
receipt = dict(render_window=window.GetClassName(), vtk_version=vtk.vtkVersion.GetVTKVersion(),
               graphics_capabilities=capabilities, backend='Mesa llvmpipe software OpenGL')
assert 'llvmpipe' in capabilities.lower(), capabilities
capture = vtk.vtkWindowToImageFilter(); capture.SetInput(window); capture.ReadFrontBufferOff(); capture.Update()
writer = vtk.vtkPNGWriter(); writer.SetFileName(str(out/'probe.png')); writer.SetInputConnection(capture.GetOutputPort()); writer.Write()
with Image.open(out/'probe.png').convert('RGB') as image:
    variance = ImageStat.Stat(image).var
    assert image.size == (64, 64) and max(variance) > 1, 'Blank software render'
receipt.update(resolution=[64,64], rgb_variance=variance, nonblank=True,
               image_sha256=hashlib.sha256((out/'probe.png').read_bytes()).hexdigest())
(out/'capabilities.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
window.Finalize()
print(json.dumps(receipt, indent=2))
'''
    with (evidence / 'probe.log').open('w', encoding='utf-8') as log:
        result = subprocess.run([sys.executable, '-c', probe, str(evidence)],
                                env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=60)
    if result.returncode:
        raise RuntimeError('Software OpenGL probe failed:\n' + (evidence / 'probe.log').read_text(encoding='utf-8'))
    receipt = {
        'url': URL, 'archive_sha256': actual_hash, 'mesa_version': '26.2.4',
        'libraries_sha256': {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in FILES},
        'deployment': 'Process-local DLL preload before VTK import; no system or Python installation changed',
        'settings': settings,
    }
    (evidence / 'mesa-source.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    if os.environ.get('GITHUB_ENV'):
        with Path(os.environ['GITHUB_ENV']).open('a', encoding='utf-8') as output:
            for key, value in settings.items():
                if '\n' in value or '\r' in value:
                    raise ValueError('Unexpected newline in CI environment path')
                output.write(f'{key}={value}\n')
    capabilities=json.loads((evidence / 'capabilities.json').read_text(encoding='utf-8'))
    print(json.dumps({**capabilities, 'graphics_capabilities': capabilities['graphics_capabilities'].split('OpenGL extensions:')[0]},indent=2))
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--archive', type=Path)
    args = parser.parse_args()
    prepare(args.directory, args.evidence, args.archive)
