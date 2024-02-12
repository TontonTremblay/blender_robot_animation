import os
import argparse
import sys
from omegaconf import OmegaConf


with open("path.txt", "w") as file:
    paths = sys.path
    current_path = os.path.dirname(os.path.realpath(__file__))
    paths.append(f'{current_path}/blender_scripts/')
    file.write(str(paths))

parser = argparse.ArgumentParser(description='Renders glbs')
parser.add_argument(
    '--config', type=str, default='configs/base.yaml',
    help='path to blender executable')


opt = parser.parse_args()


config_file = OmegaConf.load(opt.config)



render_cmd = f'PYTHONPATH=/Users/jtremblay/miniconda3/bin/python {config_file.blender_path} -b --python-use-system-env -P blender_scripts/urdf_rendering.py -- --config {opt.config}' 


# render_cmd = render_cmd + ' > tmp.out'

print(render_cmd)
os.system(render_cmd)


