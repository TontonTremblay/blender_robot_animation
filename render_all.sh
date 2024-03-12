# python render_mac.py --config configs/allegro.yaml
# cd renders
# zip -r allegro.zip allegro/ 
# rm -rf allegro/
# cd ../

# python render_mac.py --config configs/animal.yaml
# cd renders
# zip -r animal.zip animal/ 
# rm -rf animal/
# cd ../

# python render_mac.py --config configs/icub.yaml
# cd renders
# zip -r icub.zip icub/ 
# rm -rf icub/
# cd ../

# python render_mac.py --config configs/jaco.yaml
# cd renders
# zip -r jaco.zip jaco/ 
# rm -rf jaco/
# cd ../

# python render_mac.py --config configs/baxter.yaml
# cd renders
# zip -r baxter.zip baxter/ 
# rm -rf baxter/
# cd ../


python render_mac.py --config configs/ur5_normal.yaml
cd renders
zip -r ur5_normal.zip ur5_normal/ 
rm -rf ur5_normal/
cd ../


python render_mac.py --config configs/ur5_fast_robot.yaml
cd renders
zip -r ur5_fast_robot.zip ur5_fast_robot/ 
rm -rf ur5_fast_robot/
cd ../


python render_mac.py --config configs/ur5_slow_camera.yaml
cd renders
zip -r ur5_slow_camera.zip ur5_slow_camera/ 
rm -rf ur5_slow_camera/
cd ../
