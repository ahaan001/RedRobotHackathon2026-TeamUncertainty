"""
Line-sensor bracket: one printed part that bolts under the baseplate front edge and
hangs the QTR-8RC (sensors facing down) at QTR_SENSE_HEIGHT above the floor.

Frame: X lateral, Y forward (+Y ahead of the baseplate front edge at Y=0), Z up from floor.
Height is set by params (BASEPLATE_UNDER_H, QTR_SENSE_HEIGHT); shim with washers if off.
"""
import cadquery as cq
import params as P
from common import export_stl, export_step, print_mass_g, bbox_str

FLANGE_W, FLANGE_D, FLANGE_T = 56.0, 20.0, 4.0   # spans the two inner front holes at y = +-21.1
HOLE_Y = P.BASEPLATE_FRONT_HOLES_INNER[1][1]          # 21.1
HOLE_X = P.BASEPLATE_FRONT_HOLES_INNER[1][0]          # 12.6 behind the front edge
WEB_T = 4.0
SHELF_T = 3.0
SHELF_W = P.QTR_BOARD_L + 6.0
SHELF_D = P.QTR_BOARD_W + 6.0

shelf_bottom_z = P.QTR_SENSE_HEIGHT + P.QTR_COMPONENT_H + P.QTR_BOARD_T   # board top sits here
shelf_top_z = shelf_bottom_z + SHELF_T
flange_bottom_z = P.BASEPLATE_UNDER_H - FLANGE_T
web_h = P.BASEPLATE_UNDER_H - shelf_top_z + SHELF_T

if web_h < 4:
    raise SystemExit(f"Baseplate too low for this bracket (web {web_h:.1f} mm); raise QTR height or change design")


def build():
    # Top flange under the baseplate: two tabs at y = +-21.1 reaching the existing holes
    # 12.6 mm behind the front edge, with a cutout between them for the caster housing.
    flange = (cq.Workplane("XY").box(FLANGE_W, FLANGE_D, FLANGE_T, centered=(True, False, False))
              .translate((0, -FLANGE_D, flange_bottom_z)))
    notch = cq.Workplane("XY").box(28.0, FLANGE_D - 6.0, FLANGE_T * 3).translate((0, -FLANGE_D + (FLANGE_D - 6.0) / 2 - 0.01, flange_bottom_z + FLANGE_T / 2))
    flange = flange.cut(notch)
    flange = (flange.faces(">Z").workplane()
              .pushPoints([(-HOLE_Y, -HOLE_X), (HOLE_Y, -HOLE_X)])
              .hole(P.M3_CLEAR))

    # Vertical web on the front face of the baseplate
    web = (cq.Workplane("XY").box(FLANGE_W, WEB_T, web_h, centered=(True, False, False))
           .translate((0, 0, shelf_top_z - SHELF_T)))

    # Shelf ahead of the web; the QTR bolts to its underside, header pins poke up through the window
    shelf = (cq.Workplane("XY").box(SHELF_W, SHELF_D, SHELF_T, centered=(True, False, False))
             .translate((0, WEB_T, shelf_bottom_z)))
    shelf = (shelf.faces(">Z").workplane()
             .center(0, WEB_T + SHELF_D / 2)
             .rect(P.QTR_BOARD_L - 10.0, 5.0).cutThruAll())
    shelf = (shelf.faces(">Z").workplane()
             .center(0, WEB_T + SHELF_D / 2)
             .pushPoints([(-P.QTR_HOLE_SPACING / 2, 0), (P.QTR_HOLE_SPACING / 2, 0)])
             .slot2D(P.M2_CLEAR + 4.0, P.M2_CLEAR, angle=0).cutThruAll())

    part = flange.union(web).union(shelf)
    # gussets between web and shelf
    for sx in (-1, 1):
        g = (cq.Workplane("YZ").polyline([(WEB_T, shelf_top_z), (WEB_T + 12, shelf_top_z), (WEB_T, shelf_top_z + 8)])
             .close().extrude(3.0).translate((sx * (FLANGE_W / 2 - 3.0) - (0 if sx < 0 else 3.0) + (3.0 if sx < 0 else 0), 0, 0)))
        part = part.union(g)
    return part


if __name__ == "__main__":
    part = build()
    export_stl(part, "sensor_bracket")
    export_step(part, "sensor_bracket")
    print(f"sensor_bracket: {bbox_str(part)}, ~{print_mass_g(part):.0f} g PLA, "
          f"sensor face at {P.QTR_SENSE_HEIGHT} mm, web {web_h:.1f} mm")
