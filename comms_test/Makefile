current_dir = $(shell pwd)

build:
	docker build \
		-t comms_test:latest \
		.

stop:
	@docker stop comms_tester || true && docker rm comms_tester || true
run:
	@docker stop comms_tester || true && docker rm comms_tester || true
	docker run \
		--detach \
		-e ROS_IP=localhost \
		-e ROS_MASTER_URI="http://localhost:11311" \
		-v /dev:/dev \
		-v $(current_dir)/scripts:/root/ros_ws/src/scripts \
		--privileged \
		--network host \
		--name comms_tester \
		-it \
		comms_test:latest


run-roscore:
	docker exec comms_tester bash -c "source /root/ros_ws/devel/setup.bash && roscore"

run-robot_interface:
	docker exec comms_tester bash -c "source /root/ros_ws/devel/setup.bash && python3 /root/ros_ws/src/scripts/robot_interface.py"

debug: 
	docker exec -it comms_tester bash -c "source /root/ros_ws/devel/setup.bash && bash && cd /root/ros_ws/src/scripts"
