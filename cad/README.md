# cad/

Parametric CAD in CadQuery. `params.py` holds every dimension; the part scripts build from it.

    pip install cadquery trimesh matplotlib
    python build_all.py        # regenerates out/*.stl *.step *.dxf and out/contact_sheet.png
    python assembly_check.py   # kinematic side views at pickup, placement and stow

Fabrication list and quantities: `BOM.md`. Order of assembly and pre-flight: `ASSEMBLY.md`.
Pipe dimensions are placeholders until measured; see the top of `params.py`.
