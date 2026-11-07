"""Independent checks that holes exist where params say they should. Run after build_all."""
import math
import params as P
from common import circle_report
import mast, arm, gripper, sensor_bracket, plow

ok = True
def expect(cond, msg):
    global ok
    print(("  OK   " if cond else "  FAIL ") + msg)
    ok = ok and cond

# mast left: 3 boss holes + 12 tab slots (each slot = 2 arcs) + 2 lightening; right: 3 pivot + 2 lightening
cl = circle_report(mast.side_plate(True), "mast_side_left")
expect(any(abs(r - (P.SERVO_BOSS_D + 0.6) / 2) < 0.05 and abs(x - mast.MAST_PIVOT_X) < 0.1 and abs(y - mast.pivot_y(165.0)) < 0.1 for x, y, r in cl), "left plate boss hole at pivot 165")
expect(any(abs(y - mast.pivot_y(265.0)) < 0.1 and abs(r - (P.SERVO_BOSS_D + 0.6) / 2) < 0.05 for x, y, r in cl), "left plate boss hole at pivot 265")
cr = circle_report(mast.side_plate(False), "mast_side_right")
expect(sum(1 for x, y, r in cr if abs(r - P.M3_CLEAR / 2) < 0.05) == 3, "right plate has 3 pivot holes")
cb = circle_report(mast.base_plate(), "mast_base")
expect(any(abs(x - (P.MAST_CENTER_REAR_X - 113.3)) < 0.1 and abs(y - 42.9) < 0.1 for x, y, r in cb), "base hole for rear pattern (113.3, 42.9)")
expect(any(abs(x - (P.MAST_CENTER_FRONT_X - 42.9)) < 0.1 and abs(y - 63.5) < 0.1 for x, y, r in cb), "base hole for front pattern (42.9, 63.5)")

ca = circle_report(arm.arm_plate(), "arm_plate")
expect(any(abs(x - P.ARM_LENGTH) < 0.1 and abs(y) < 0.1 and abs(r - P.M3_CLEAR / 2) < 0.05 for x, y, r in ca), f"arm gripper pivot hole at L={P.ARM_LENGTH}")
expect(any(abs(x - P.ARM_LENGTH_ALT) < 0.1 and abs(y) < 0.1 for x, y, r in ca), f"arm gripper pivot hole at L={P.ARM_LENGTH_ALT}")
expect(any(abs(x) < 0.1 and abs(y) < 0.1 and abs(r - P.SERVO_HORN_HUB_D / 2) < 0.05 for x, y, r in ca), "arm hub hole at pivot")
expect(sum(1 for x, y, r in ca if abs(r - P.SERVO_HORN_HOLE_D / 2) < 0.05) == 8, "arm has 8 horn screw holes")

cc = circle_report(gripper.carrier(), "gripper_carrier")
expect(any(abs(x - gripper.S / 2) < 0.1 and abs(y) < 0.1 and abs(r - (P.SERVO_BOSS_D + 0.6) / 2) < 0.05 for x, y, r in cc), "carrier servo boss hole at (+18, 0)")
expect(any(abs(x + gripper.S / 2) < 0.1 and abs(y) < 0.1 and abs(r - P.M3_CLEAR / 2) < 0.05 for x, y, r in cc), "carrier idler hole at (-18, 0)")
cj = circle_report(gripper.jaw(True), "jaw_driven")
expect(sum(1 for x, y, r in cj if abs(r - P.SERVO_HORN_HOLE_D / 2) < 0.05) >= 6, "driven jaw has horn screw holes on PCD 14 and 20")

cs = circle_report(sensor_bracket.build(), "sensor_bracket")
expect(any(abs(abs(x) - 21.1) < 0.1 and abs(r - P.M3_CLEAR / 2) < 0.05 for x, y, r in cs), "bracket M3 holes at y=+-21.1")

cp = circle_report(plow.base(), "plow_base")
expect(sum(1 for x, y, r in cp if abs(abs(x) - 36.0) < 0.1) == 2, "plow bolt holes at +-36")

print("\nALL CHECKS PASSED" if ok else "\nSOME CHECKS FAILED")
raise SystemExit(0 if ok else 1)
