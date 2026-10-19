from pathlib import Path
import subprocess,tempfile,os,re
R=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as t:
 exe=str(Path(t)/('controller.exe' if os.name=='nt' else 'controller'))
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-function','-Icontrol/lib/production_control/src','-Icommunication/include','verification/controller_test.cpp','control/lib/production_control/src/production_control.cpp','control/lib/production_control/src/competition_actuators.cpp','-o',exe],cwd=R,check=True)
 subprocess.run([exe],check=True)
 for p in (R/'communication/include').glob('*page*.h'):
  s=p.read_text(encoding='utf-8')
  if '<script>' in s:
   f=Path(t)/(p.stem+'.js');f.write_text(s.split('<script>')[1].split('</script>')[0],encoding='utf-8');subprocess.run(['node','--check',str(f)],check=True)
   ids=set(re.findall(r'\bid="([^"]+)"',s));refs=set(re.findall(r"(?:get|\$)\('([^']+)'\)",s));assert not refs-ids,(p,refs-ids)
 a=(R/'control/lib/boat_protocol/src/boat_protocol.h').read_bytes();b=(R/'communication/lib/boat_protocol/src/boat_protocol.h').read_bytes();assert a==b
 s=(R/'communication/src/main.cpp').read_text(encoding='utf-8');assert '"/diagnostics"' not in s and '"/api/diagnostics"' not in s
 assert not re.search(r'parseFloatArgument\("\w+_cutoff_hz"',s)
 assert 'filterCommand(' not in (R/'control/lib/production_control/src/production_control.cpp').read_text(encoding='utf-8')
print('JS syntax, DOM targets, protocol agreement, LPF/diagnostics absence passed')
