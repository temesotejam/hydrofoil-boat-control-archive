"""Package only firmware projects; merge four builds and produce the installer site."""
from pathlib import Path
import argparse,json,hashlib,shutil,subprocess,zipfile
from verify_archive import ROOT,verify

def zip_sources(day,version,nodes,out):
 name='hydrofoil-'+day+('-'+nodes[0] if len(nodes)==1 else '-program')
 text=f'''水中翼ボート {version['label']} プログラム\n原典: https://github.com/temesotejam/2026_8_6\n対象コミット: {version['commit']}\n\n通信側と制御側は同じ日の版を組み合わせてください。\nC++の改行・インデント・空白を整えています。platformio.iniは原典のままです。\n'''
 if version.get('adaptation'):
  text+='1日目は大会コースA→B→C→D→E→F→G→H→C→B→Aの一括送信に対応します。その他の制御・時間設定は1日目を維持しています。\n'
 for node in nodes:
  command='.' if len(nodes)==1 else node
  text+=f'\n{node}: PlatformIOで{command}を開くか、展開先で python -m platformio run -d {command} -e production を実行してください。\n'
 text+='\n初回ビルドではPlatformIOが外部依存を取得します。制御側のSparkFunライブラリは原典で版が固定されていません。再ビルドしたバイナリと当日書き込んだバイナリの同一性を保証するものではありません。\n'
 with zipfile.ZipFile(out/(name+'.zip'),'w',compression=zipfile.ZIP_DEFLATED) as z:
  entries={'README.txt':text.encode()}
  for node in nodes:
   for rel in version['sha256']:
    if rel.startswith(node+'/'):
     path=rel if len(nodes)>1 else rel[len(node)+1:]
     entries[path]=(ROOT/day/rel).read_bytes()
  for path,data in sorted(entries.items()):
   zi=zipfile.ZipInfo(name+'/'+path,date_time=(2026,8,9,0,0,0));zi.compress_type=zipfile.ZIP_DEFLATED;zi.external_attr=0o100644<<16;z.writestr(zi,data)
 # A ZIP must be independently buildable and exclude the archive website/tests/other day.
 with zipfile.ZipFile(out/(name+'.zip')) as z:
  names=z.namelist();assert not any('/tests/' in n or '/web/' in n or '/.pio/' in n or '/.github/' in n for n in names)
  for node in nodes:assert name+'/'+(node+'/' if len(nodes)>1 else '')+'platformio.ini' in names

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--sources-only',action='store_true');args=parser.parse_args()
 info=verify();site=ROOT/'site'
 if site.exists():shutil.rmtree(site)
 shutil.copytree(ROOT/'web',site)
 (site/'downloads').mkdir();builds={}
 for day,version in info['versions'].items():
  for nodes in [('control','communication'),('control',),('communication',)]:zip_sources(day,version,nodes,site/'downloads')
  variant=version.get('adaptation',{}).get('id','')
  build_id=version['commit'][:7]+('-'+variant if variant else '')
  builds[day]={'commit':version['commit'],'variant':variant,'label':version['label'],'firmware':{}}
  if args.sources_only:continue
  for node in ('control','communication'):
   build=ROOT/day/node/'.pio/build/production';dest=site/day;dest.mkdir(exist_ok=True)
   core=Path.home()/'.platformio/packages';tool=core/'tool-esptoolpy/esptool.py';boot=core/'framework-arduinoespressif32/tools/partitions/boot_app0.bin'
   parts=[(0,build/'bootloader.bin'),(0x8000,build/'partitions.bin'),(0xe000,boot),(0x10000,build/'firmware.bin')]
   assert tool.is_file()
   for offset,path in parts:assert path.is_file() and path.stat().st_size>0,path
   output=dest/(node+'.bin')
   subprocess.run(['python',str(tool),'--chip','esp32s3','merge_bin','-o',str(output),'--flash_mode','qio','--flash_freq','80m','--flash_size','8MB',*[item for offset,path in parts for item in (hex(offset),str(path))]],check=True)
   manifest={'name':f'水中翼ボート {version["label"]} '+('制御側' if node=='control' else '通信側'),'version':build_id,'new_install_prompt_erase':True,'new_install_improv_wait_time':0,'builds':[{'chipFamily':'ESP32-S3','parts':[{'path':node+'.bin','offset':0}]}]}
   (dest/(node+'.json')).write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
   digest=hashlib.sha256(output.read_bytes()).hexdigest();(dest/(node+'.bin.sha256')).write_text(digest+'  '+node+'.bin\n');builds[day]['firmware'][node]=digest
 checks=[]
 for p in sorted((site/'downloads').glob('*.zip')):checks.append(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name)
 (site/'downloads/SHA256SUMS.txt').write_text('\n'.join(checks)+'\n')
 (site/'build-info.json').write_text(json.dumps({'ready':not args.sources_only,'versions':builds},ensure_ascii=False,indent=2)+'\n')
 (site/'.nojekyll').touch()
 print('Built '+str(site))
if __name__=='__main__':main()
