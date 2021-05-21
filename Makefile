.POSIX:

PREFIX ?= ./
VENV ?= $(PREFIX)rcp/

PYTHON_X ?= python3
PIP_X ?= pip

.SILENT: help
.PHONY: help # ----- (help)  print this help text
help:
	@grep '^.PHONY: .* #' $(firstword $(MAKEFILE_LIST)) |\
		sed 's/\.PHONY: \(.*\) # \(.*\)/\1 # \2/' |\
		awk 'BEGIN {FS = "#"}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' 

build: init install

.PHONY: rcp # ----- (combo) build rcp
rcp: build

.PHONY: init # (build)     initialize python venv
init:
	$(PYTHON_X) -m venv $(VENV)

.ONESHELL:
.PHONY: install # (build)     source python venv and install dependencies
install:
	. $(VENV)bin/activate
	$(PYTHON_X) -m $(PIP_X) install --upgrade $(PIP_X)
	$(PIP_X) install -r $(VENV)requirements.txt

.ONESHELL:
.PHONY: resolve # (build)     source python venv and update requirements.txt
resolve:
	. $(VENV)bin/activate
	$(PIP_X) freeze > $(VENV)requirements.txt

.PHONY: clean # (build)     clean python venv
clean:
	rm -rf $(VENV)bin
	rm -rf $(VENV)include
	rm -rf $(VENV)lib
	rm -rf $(VENV)lib64
	rm -f $(VENV)pyvenv.cfg
	rm -rf $(VENV)__pycache__
	rm -rf $(VENV)share
