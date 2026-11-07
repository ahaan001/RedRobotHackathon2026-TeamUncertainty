"""Regenerate every export from params.py. Run: python build_all.py"""
import subprocess
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
for script in ("sensor_bracket.py", "mast.py", "arm.py", "gripper.py", "plow.py", "render.py"):
    print(f"=== {script}")
    r = subprocess.run([sys.executable, os.path.join(HERE, script)], cwd=HERE)
    if r.returncode != 0:
        sys.exit(f"{script} failed")
print("done. Review out/contact_sheet.png, then run assembly_check.py for the kinematic views.")
