.POSIX:

PREFIX ?= ./rcp/

DESTDIR_CONFIG ?= config/

DESTDIR_TEST ?= test/
DESTDIR_TEST_EXAMPLES ?= examples/
DESTDIR_TEST_ENV_NOROS ?= host/
DESTDIR_TEST_ENV_ROS ?= workspace/src/ros_usage_example/ros_usage_example/

DESTDIR_OUT ?= out/

DOCKER_X ?= docker
DOCKER_COMPOSE_X ?= docker-compose
PYTHON_X ?= python
RCP_X ?= rcp.py

.SILENT: help
.PHONY: help # (help)      print this help text
help:
	@grep '^.PHONY: .* #' $(firstword $(MAKEFILE_LIST)) |\
		sed 's/\.PHONY: \(.*\) # \(.*\)/\1 # \2/' |\
		awk 'BEGIN {FS = "#"}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' 

# --------------------------------------------------
# RCP INIT

r_build: r_init r_install
.PHONY: rcp # ----- (combo)[rcp] build
rcp: r_build

.PHONY: r_init # [rcp] initialize python venv
r_init:
	$(PYTHON_X) -m venv $(RCP_PATH)

.ONESHELL:
.PHONY: r_install # [rcp] source python venv and install dependencies
r_install:
	. $(RCP_PATH)/bin/activate
	$(PYTHON_X) -m $(PIP_X) install --upgrade $(PIP_X)
	$(PIP_X) install -r $(RCP_PATH)/requirements.txt

.ONESHELL:
.PHONY: r_resolve # [rcp] source python venv and update requirements.txt
r_resolve:
	. $(RCP_PATH)/bin/activate
	$(PIP_X) freeze > $(RCP_PATH)/requirements.txt

.PHONY: r_clean # [rcp] clean python venv
r_clean:
	-rm -rf $(RCP_PATH)/bin
	-rm -rf $(RCP_PATH)/include
	-rm -rf $(RCP_PATH)/lib
	-rm -rf $(RCP_PATH)/lib64
	-rm -f $(RCP_PATH)/pyvenv.cfg
	-rm -rf $(RCP_PATH)/__pycache__
	-rm -rf $(RCP_PATH)/share

# --------------------------------------------------
# RCP BUILD

.SILENT: build
.PHONY: build # (general)   generate libraries code
build:
	$(PYTHON_X) $(PREFIX)$(RCP_X) $(DESTDIR_CONFIG)freenove.yml

.SILENT: clean
.PHONY: clean # (general)   clean env
clean:
	sudo chown -R ${USER}:${USER} $(PREFIX)$(DESTDIR_TEST)workspace
	rm -rf $(PREFIX)$(DESTDIR_TEST)workspace/build
	rm -rf $(PREFIX)$(DESTDIR_TEST)workspace/install
	rm -rf $(PREFIX)$(DESTDIR_TEST)workspace/log
	$(DOCKER_COMPOSE_X) -f $(PREFIX)$(DESTDIR_TEST)docker-compose.yml down
	$(DOCKER_X) system prune -f
	$(DOCKER_X) volume prune -f
	$(DOCKER_X) network prune -f


.PHONY: build_sensors # (sensors)   parse, generate, and update examples
build_sensors: build clean
	# ros2
	cp -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_EXAMPLES)sensors/ros_usage_example.py $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_ROS)main.py
	cp -rf $(PREFIX)$(DESTDIR_OUT)freenove_4WD_smart_car/ros $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_ROS)
	rm -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_ROS)__pycache__
	# noros
	cp -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_EXAMPLES)sensors/noros_usage_example.py $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)main.py
	cp -rf $(PREFIX)$(DESTDIR_OUT)freenove_4WD_smart_car/noros $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)
	rm -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)noros/__pycache__

.PHONY: build_actuators # (actuators) parse, generate, and update examples
build_actuators: build
	# ros2
	cp -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_EXAMPLES)actuators/ros_usage_example.py $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_ROS)main.py
	cp -rf $(PREFIX)$(DESTDIR_OUT)freenove_4WD_smart_car/ros $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_ROS)
	rm -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_ROS)__pycache__
	# noros
	cp -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_EXAMPLES)actuators/noros_usage_example.py $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)main.py
	cp -rf $(PREFIX)$(DESTDIR_OUT)freenove_4WD_smart_car/noros $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)
	rm -rf $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)noros/__pycache__

.SILENT: exec_remote_ros
.PHONY: exec_remote_ros # (exec)      exec ros example in distributed env
exec_remote_ros:
	$(DOCKER_COMPOSE_X) -f ./test/docker-compose.yml up

.SILENT: exec_remote_rpi
.PHONY: exec_remote_rpi # (exec)      exec noros example in distributed env
exec_remote_rpi:
	$(PYTHON_X) $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)noros_usage_example.py

.SILENT: exec_local
.PHONY: exec_local # (exec)      exec ros/noros examples in local env
exec_local:
	$(DOCKER_COMPOSE_X) -f ./test/docker-compose.yml up
	$(PYTHON_X) $(PREFIX)$(DESTDIR_TEST)$(DESTDIR_TEST_ENV_NOROS)noros_usage_example.py
