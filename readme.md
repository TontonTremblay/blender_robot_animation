# Animated robot with rotating camera

![data_gen](https://github.com/TontonTremblay/blender_robot_animation/assets/5629088/4cdb74ff-505b-47f0-8f54-b90a1702d419)



You need to specify the path to blender in the config file, `configs/base.yaml`. This is where the absolute path to this code folder should be set. Each robot file loads this to populate the base paths. 

The robot assets are downloaded as submodules: 
```
git submodule init; git submodule update
```

You can download hdri env maps [here](https://drive.google.com/file/d/1lp36MgTlS4OFaH0vdsTFhyGFJpQDY2YX/view?usp=drive_link).

You can download the texture assets with: 
```
python download_textures.py
```

Check the config file where the data will be stored. But you should be able to run: 
```
python render_mac.py --config baxter.yaml
```

# Installation 
In order to include pip installed python packages, you need to have your "activated" python to be 3.10. We use a trick where we take the python paths and add them to the blender python, so you can use your python packages.  

You also need blender 4.0. It might work with other versions but I am not sure. 

```
pip install -r requirements.txt
```
