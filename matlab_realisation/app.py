import subprocess
import glob

w = 5
# p1 = [690 -151 162]
# p2 = [465 377 162]
l = 10
rTorch = 50
detail_num = 2

command = f'matlab -batch "w={w}; l={l}; rTorch={rTorch}; d_num={detail_num}; x1=690; y1=-151; z1=162; x2=465; y2=377; z2=162; matlab_app;"'
process = subprocess.run(command, capture_output=True, text=True)
print(process.returncode)
print(process.stdout)
print(process.stderr)