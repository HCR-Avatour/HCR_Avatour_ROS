include .env
export $(shell sed 's/=.*//' .env)

build:
	docker build \
		-t prl_p3at_ros_driver:latest \
		.

stop:
	@docker stop p3at_ros_driver || true && docker rm p3at_ros_driver || true
run:
	@sudo chmod 666 ${P3AT_USB_PORT}
	docker run \
		-e ROS_IP=${MY_IP} \
		-e ROS_MASTER_URI="http://${MY_IP}:11311" \
		-e P3AT_USB_PORT=${P3AT_USB_PORT} \
		-v /dev:/dev \
		--privileged \
		--network host \
		--name p3at_ros_driver \
		-it \
		prl_p3at_ros_driver:latest

run-windows:
	docker run \
		-e ROS_IP=${MY_IP} \
		-e ROS_MASTER_URI="http://${MY_IP}:11311" \
		--isolation=process \
		--device="class/86E0D1E0-8089-11D0-9CE4-08003E301F73" \
		--privileged \
		--network host \
		--name p3at_ros_driver \
		-it \
		prl_p3at_ros_driver:latest