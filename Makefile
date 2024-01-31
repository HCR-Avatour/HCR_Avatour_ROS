include .env
export $(shell sed 's/=.*//' .env)
current_dir = $(shell pwd)

build:
	docker build \
		-t prl_p3at_ros_driver:latest \
		.

stop:
	@docker stop p3at_ros_driver || true && docker rm p3at_ros_driver || true
run:
	@docker stop p3at_ros_driver || true && docker rm p3at_ros_driver || true
	@sudo chmod 777 -R ${P3AT_USB_PORT}
	docker run \
		-e ROS_IP=${MY_IP} \
		-e ROS_MASTER_URI="http://${MY_IP}:11311" \
		-e P3AT_USB_PORT=${P3AT_USB_PORT} \
		-v /dev:/dev \
		-v $(current_dir)/scripts:/root/ros_ws/src/scripts \
		--privileged \
		--network host \
		--name p3at_ros_driver \
		-it \
		prl_p3at_ros_driver:latest


run-roscore:
	docker exec p3at_ros_driver bash -c "source /root/ros_ws/devel/setup.bash && roscore"

run-robot_interface:
	docker exec p3at_ros_driver bash -c "source /root/ros_ws/devel/setup.bash && python3 /root/ros_ws/src/scripts/robot_interface.py"

run-keyboard:
	docker exec p3at_ros_driver bash -c "source /root/ros_ws/devel/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel"