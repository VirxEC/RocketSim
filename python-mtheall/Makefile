.PHONY: all build wheel install clean

PYTHON ?= python3

all: build

build:
	$(PYTHON) ../setup.py build --debug -j 4

wheel: build
	$(PYTHON) ../setup.py bdist_wheel --py-limited-api=cp34

install: wheel
	$(PYTHON) -m pip install --force-reinstall --user ./dist/RocketSim-2.1.0.post3-cp34-abi3-linux_x86_64.whl

clean:
	$(RM) -r build/ dist/ RocketSim.egg-info/
