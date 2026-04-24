import sys
import subprocess

out = subprocess.check_output([sys.executable, "-m", "pip", "list", "--format=json"])
print(out)
