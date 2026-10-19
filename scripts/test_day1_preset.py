"""Exercise UI configuration -> real COMM packet -> unchanged day1 controller."""
from pathlib import Path
import os,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
    exe=str(Path(tmp)/('day1-course.exe' if os.name=='nt' else 'day1-course'))
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-DARDUINO','-include','cstring',
        '-Iscripts/tests/stubs','-Iday1/tests/stubs','-Iday1/communication/include',
        '-Iday1/communication/lib/boat_protocol/src','-Iday1/control/lib/production_control/src',
        'scripts/tests/day1_course_test.cpp','day1/communication/src/gnss_receiver.cpp',
        'day1/communication/lib/boat_protocol/src/boat_protocol.cpp',
        'day1/control/lib/production_control/src/production_control.cpp',
        'day1/control/lib/production_control/src/competition_actuators.cpp','-o',exe],cwd=ROOT,check=True)
    subprocess.run([exe],cwd=ROOT,check=True)
print('day1 preset: UI selection, 11-point packet/CRC, sequential tracking, final stop and single target passed')
