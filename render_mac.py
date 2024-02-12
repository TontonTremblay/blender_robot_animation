import os
import argparse
import sys


with open("path.txt", "w") as file:
    paths = sys.path
    current_path = os.path.dirname(os.path.realpath(__file__))
    paths.append(f'{current_path}/blender_scripts/')
    file.write(str(paths))

parser = argparse.ArgumentParser(description='Renders glbs')
parser.add_argument(
    '--blender_root', type=str, default='/Applications/Blender.app/Contents/MacOS/Blender',
    help='path to blender executable')

parser.add_argument(
    '--config', type=str, default='configs/base.yaml',
    help='path to blender executable')


opt = parser.parse_args()




render_cmd = f'PYTHONPATH=/Users/jtremblay/miniconda3/bin/python {opt.blender_root} -b --python-use-system-env -P blender_scripts/urdf_rendering.py -- --config {opt.config}' 


# render_cmd = render_cmd + ' > tmp.out'

print(render_cmd)
os.system(render_cmd)


