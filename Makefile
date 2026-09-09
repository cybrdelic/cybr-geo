.PHONY: test doctor motor media drawings release

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
