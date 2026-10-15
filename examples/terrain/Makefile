.PHONY: test smoke preview
test:
	python -m pytest -q
smoke:
	terrain demo all --quality smoke --grid 33 --duration 2 --threads 2
preview:
	terrain demo badlands --quality preview
