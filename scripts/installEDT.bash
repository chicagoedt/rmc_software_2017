rosdep update
rosdep install --from-paths ./ --ignore-src --rosdistro indigo -y

cd .git/hooks

echo "#!/bin/bash" >> post-receive
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo -y" >> post-receive
echo "#!/bin/bash" >> post-update
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo -y" >> post-update

sudo chmod +x post-receive
sudo chmod +x post-update

cd ../../../../

catkin_make clean
catkin_make


echo "Putting setup.bash into your .bashrc..."
cd devel/

path="$(pwd)"

echo "source $path/setup.bash" >> ~/.bashrc

echo "Sourcing the ~/.bashrc..."
source ~/.bashrc

echo "Executing 'roscd'..." 
roscd
echo "You should now be in the /devel/ folder."

echo "Now grabbing rmc's gazebo meshes from the EDT wiki..."
cd ..
cd src/Software_RMC/simulation/LunArena2

wget http://wiki.chicagoedt.org/images/7/7e/Meshes-LunArena2.tar.gz
tar zxvf Meshes-LunArena2.tar.gz
rm Meshes-LunArena2.tar.gz

roscd rmc_simulation

wget http://wiki.chicagoedt.org/images/c/ce/Meshes-rmc-simulation.tar.gz
tar zxvf Meshes-rmc-simulation.tar.gz
rm Meshes-rmc-simulation.tar.gz

roscd rmc_simulation/..
if [ ! -d ~/.gazebo/models/ ]; then
        mkdir -p ~/.gazebo/models/
        cp -rf rmc_simulation ~/.gazebo/models
        cp -rf LunArena2 ~/.gazebo/models/
        echo "Made directory"
else
        cp -rf rmc_simulation ~/.gazebo/models
        ls -l ~/.gazebo/models/scipio_simulation
        echo "Tried to copy to scipio_simulation"
        cp -rf LunArena2 ~/.gazebo/models/
        ls -l ~/.gazebo/models/grass
        echo "Tried to copy to grass"
fi

echo "Finished Installing EDT's RMC Repository."
