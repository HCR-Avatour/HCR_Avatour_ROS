include .env
export $(shell sed 's/=.*//' .env)

build:
	DOCKER_BUILDKIT=1 docker build \
		-t prl_p3at_ros_driver:latest \
		.

run:
	@docker stop p3at_ros_driver || true && docker rm p3at_ros_driver || true
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