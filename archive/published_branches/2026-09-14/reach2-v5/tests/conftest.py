from pathlib import Path
import os
import pytest

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MECHANISM_LAB_ROOT',str(ROOT))
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('LP_NUM_THREADS','2')

@pytest.fixture(scope='session')
def root():return ROOT

@pytest.fixture(scope='session')
def flange():
    from mechanism_lab.models.example import build
    return build()

@pytest.fixture(scope='session')
def motor(root):
    from mechanism_lab.core import load_cache
    from mechanism_lab.models.motor import motor_pose,build
    cache=root/'outputs/m8325s/cache'
    a=load_cache(cache) if (cache/'manifest.json').exists() else build()
    a.motion_function=motor_pose
    return a

@pytest.fixture(scope='session')
def drivetrain(root):
    from mechanism_lab.core import load_cache
    from mechanism_lab.models.drivetrain import pose,build
    cache=root/'outputs/drivetrain/cache'
    a=load_cache(cache) if (cache/'manifest.json').exists() else build()
    a.motion_function=pose
    return a
