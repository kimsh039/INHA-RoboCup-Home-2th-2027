#!/usr/bin/env python3
"""Send one CSV target pose to Gazebo. CSV values are commands, never measurements."""
import argparse,csv,math,subprocess
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--table',type=Path,required=True);p.add_argument('--sample',required=True);a=p.parse_args()
with a.table.open(newline='') as f:rows=list(csv.DictReader(f))
matches=[r for r in rows if r['sample_id']==a.sample]
if len(matches)!=1:raise SystemExit('Sample ID must occur exactly once')
row=matches[0];values=[float(row[f'joint{i}_rad']) for i in range(1,7)]
if not all(math.isfinite(x) for x in values):raise SystemExit('Angles must be finite radians')
for i,q in enumerate(values,1):subprocess.run(['gz','topic','-t',f'/model/robocup/joint/piper_joint{i}/0/cmd_pos','-m','gz.msgs.Double','-p',f'data: {q:.17g}'],check=True)
print('Target pose sent. Wait for settling, check the tag, then run capture_pose.py to save measured joints.')
