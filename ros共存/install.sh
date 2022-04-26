if [ ! -f /usr/bin/terminator ]; then
	sudo apt-get update
	sudo apt install terminator
fi



sudo mkdir -p /usr/share/icons/ros

sudo cp ./ros.png /usr/share/icons/ros/ros.png
sudo chmod 777 /usr/share/icons/ros/ros.png
sudo cp ./ros.desktop /usr/share/applications/ros.desktop
sudo chmod 777 /usr/share/applications/ros.desktop

sudo cp ./ros2.png /usr/share/icons/ros/ros2.png
sudo chmod 777 /usr/share/icons/ros/ros2.png
sudo cp ./ros2.desktop /usr/share/applications/ros2.desktop
sudo chmod 777 /usr/share/applications/ros2.desktop



for user in `ls /home`; do 
	dir_home=/home/${user}
	if [ -d ${dir_home} ]; then
		echo "install to user: ${user} dir: ${dir_home}"

		if [ ! -f ${dir_home}/.ros_setup.bash ]; then
			sudo cp ./ros_setup.bash ${dir_home}/.ros_setup.bash
			sudo chmod 777 ${dir_home}/.ros_setup.bash
			sudo chown ${user}:${user} ${dir_home}/.ros_setup.bash
		fi

		if [ ! -f ${dir_home}/.ros2_setup.bash ]; then
			sudo cp ./ros2_setup.bash ${dir_home}/.ros2_setup.bash
			sudo chmod 777 ${dir_home}/.ros2_setup.bash
			sudo chown ${user}:${user} ${dir_home}/.ros2_setup.bash
		fi
		
		sudo mkdir -p ${dir_home}/.config/terminator
		sudo cp ./config ${dir_home}/.config/terminator/config
		sudo chmod 777 ${dir_home}/.config/terminator/config
		sudo chown -R ${user}:${user} ${dir_home}/.config
	fi
done

