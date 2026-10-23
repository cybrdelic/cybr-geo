.PHONY: test test-core test-mechanisms preview doctor motor media drawings release

ENGINEERING_TESTS := $(wildcard tests/test_engineering_*.py tests/test_arx*.py tests/test_iris*.py tests/test_fourbar*.py tests/test_linkages.py tests/test_native_replay.py tests/test_native_assembly_placement.py tests/test_native_clearance.py tests/test_reference_clock.py tests/test_part_geometry.py tests/test_manufacturing_*.py)
STUDIO_TESTS := $(wildcard studio/tests/test_*.py)

test-mechanisms:
	python -m pytest -q tests/test_construction.py tests/test_mechanism.py tests/test_motion.py tests/test_constructed.py tests/test_constructed_audit.py tests/test_construction_bundle.py tests/test_mechanism_entrypoints.py tests/test_dynamics.py tests/test_motion_envelope.py tests/test_actuation.py tests/test_contact_geometry.py tests/test_native_construction.py tests/test_shape_dag.py tests/test_causal_runtime.py tests/test_causal_counterfactual.py tests/test_physical_model.py tests/test_physical_replay.py $(ENGINEERING_TESTS) $(STUDIO_TESTS)

test-core:
	python -m pytest -q tests/test_core.py tests/test_nitinol_actuator.py tests/test_analytic_normals.py tests/test_truth_rendering.py tests/test_shared_photo_pipeline.py tests/test_preview.py

preview:
	lab preview --step

test:
	python -m pytest -q

doctor:
	lab doctor

motor:
	lab build m8325s --step
	lab render m8325s
	lab animate m8325s

media:
	python tools/build_release_media.py

drawings:
	lab blueprint m8325s
	lab blueprint drivetrain --part Drive_01_99p6_BCD_carrier_adapter
	lab blueprint drivetrain --part Drive_04_M3_face_adapter --out outputs/drivetrain/drawings/motor_face_adapter

release:
	lab pack --out dist/cybr-mechanism-lab-full.zip
	lab pack --source-only --out dist/cybr-mechanism-lab-regeneration-kit.zip
