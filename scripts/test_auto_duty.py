from pathlib import Path
import subprocess,tempfile,os,re
ROOT=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
 for day in ['day1','day2']:
  root=ROOT/day
  text=(root/'control/include/app_config.h').read_text(encoding='utf-8')
  assert re.search(r'kCompetitionAutoPropulsion\s*=\s*1\.0f',text)
  app=(root/'control/src/main.cpp').read_text(encoding='utf-8')
  assert 'competitionConfig.autoPropulsion = kCompetitionAutoPropulsion;' in app
  exe=str(Path(tmp)/(day+('.exe' if os.name=='nt' else '')))
  subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-function','-include','initializer_list','-I'+str(root/'control/lib/production_control/src'),str(ROOT/'scripts/tests/auto_duty_test.cpp'),str(root/'control/lib/production_control/src/production_control.cpp'),str(root/'control/lib/production_control/src/competition_actuators.cpp'),'-o',exe],check=True)
  subprocess.run([exe],check=True)
  print(day+': automatic request 100%, maximum scaling, ramp and stop protection passed')
