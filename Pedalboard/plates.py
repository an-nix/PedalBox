# -*- coding: utf-8 -*-
"""Create one parametric perforated plate in FreeCAD.

Set TARGET_WIDTH_MM below. The script selects the grid column count whose
resulting plate width is closest to that target while retaining the original
hole pitch and grid phase. Slots that would extend past either plate edge are
not generated.

The original pattern uses a 11.833 mm pitch in both directions, 22 rows,
6.25 mm round holes, M3 counterbored corner holes, and horizontal slots every
third row. The dimensions of the rear ribs were inferred from the STEP model
and may need adjustment for the final design.
"""

import FreeCAD as App
import Part

# --------------------------------------------------------------------------
# EDITABLE PARAMETERS
# --------------------------------------------------------------------------
PITCH = 11.833          # Grid pitch in X and Z (mm).
D_SMALL = 6.25          # Standard round-hole diameter (mm).
D_CORNER = 3.50         # Corner through-hole diameter for M3 screws (mm).
CORNER_COUNTERBORE_DIAMETER = 6.20  # Circular head recess diameter (mm).
CORNER_HEAD_DEPTH = 3.20            # Head recess depth from the top (mm).
CORNER_CHAMFER_DEPTH = 1.35         # 45-degree transition to the through-hole (mm).

# Slot dimensions. A slot spans two adjacent grid columns.
SLOT_WIDTH = 15.70      # Slot width, equal to twice the end radius (mm).
SLOT_SPAN = PITCH       # Distance between the two end centres (mm).

N_ROWS = 22             # Fixed number of hole rows.
HEIGHT_MM = 260.0       # Fixed plate height in Z (mm).
MARGIN_Z = 5.7535       # Fixed edge-to-first-row margin (mm).

MARGIN_X = 4.0          # Edge-to-first-column margin (mm).
THICKNESS_MM = 5.0      # Plate thickness in Y (mm).

CORNER_HOLE = True      # Use counterbored M3 holes at the four grid corners.

# Rear ribs. Their dimensions were inferred from the STEP model.
ADD_RIBS = True          # Set to False to omit the rear ribs.
RIB_EVERY_N_ROWS = 3     # One rib every N rows.
RIB_ROW_START = 3.5      # First rib position in row-index units.
RIB_WIDTH_Z = 2.0        # Rib width in Z (mm).
RIB_DEPTH_Y = 5.0        # Rib depth towards negative Y (mm).


# --------------------------------------------------------------------------
# HOLE-GRID CONSTRUCTION (independent from FreeCAD)
# --------------------------------------------------------------------------
def _centered_slot_starts(n_cols, prefer_right):
    """Return slot-pair starts for the phase closest to the plate centre."""
    candidates = []
    for phase in range(4):
        starts = list(range(phase, n_cols - 1, 4))
        if starts:
            centre = sum(start + 0.5 for start in starts) / len(starts)
            offset = abs(centre - (n_cols - 1) / 2.0)
            candidates.append((offset, phase, starts))

    best_offset = min(candidate[0] for candidate in candidates)
    best_candidates = [candidate for candidate in candidates
                       if abs(candidate[0] - best_offset) < 1e-9]
    return max(best_candidates, key=lambda candidate: candidate[1])[2] if prefer_right else \
        min(best_candidates, key=lambda candidate: candidate[1])[2]


def build_hole_grid(n_cols, n_rows=N_ROWS, pitch=PITCH,
                     d_small=D_SMALL, d_corner=D_CORNER,
                     corner_hole=CORNER_HOLE, plate_width=None):
    """Return hole dictionaries in a grid-centred coordinate system.

    A slot is centred between two adjacent columns and replaces the round
    holes in those columns only when it fits fully within `plate_width`.
    """

    x_center = (n_cols - 1) * pitch / 2.0
    z_center = (n_rows - 1) * pitch / 2.0

    holes = []
    for ri in range(n_rows):
        special_row = (ri % 3 == 2)
        slot_pairs = []
        if special_row:
            starts = _centered_slot_starts(n_cols, prefer_right=(ri % 6 == 2))
            slot_pairs = [(start, start + 1) for start in starts]

        z = ri * pitch - z_center
        cols_in_slot = set()
        for c1, c2 in slot_pairs:
            x_mid = ((c1 + c2) / 2.0) * pitch - x_center
            slot_half_length = (SLOT_SPAN + SLOT_WIDTH) / 2.0
            slot_fits = (plate_width is None or
                         abs(x_mid) + slot_half_length <= plate_width / 2.0)
            if slot_fits:
                cols_in_slot.add(c1)
                cols_in_slot.add(c2)
                holes.append({'x': x_mid, 'z': z, 'kind': 'slot',
                              'span': SLOT_SPAN, 'width': SLOT_WIDTH})

        for ci in range(n_cols):
            if ci in cols_in_slot:
                continue
            x = ci * pitch - x_center
            is_corner = corner_hole and ci in (0, n_cols - 1) and ri in (0, n_rows - 1)
            if is_corner:
                holes.append({'x': x, 'z': z, 'kind': 'corner', 'd': d_corner})
            else:
                holes.append({'x': x, 'z': z, 'kind': 'round', 'd': d_small})

    return holes


def columns_for_width(width_mm, pitch=PITCH, margin_x=MARGIN_X):
    """Return the column count and closest corresponding plate width."""
    usable = width_mm - 2 * margin_x
    if usable < 0:
        raise ValueError("Width is smaller than the defined margins (%.2f mm)" % (2 * margin_x))
    n_cols = int(round(usable / pitch)) + 1
    n_cols = max(n_cols, 1)
    real_width = (n_cols - 1) * pitch + 2 * margin_x
    return n_cols, real_width


# --------------------------------------------------------------------------
# FREECAD CONSTRUCTION
# --------------------------------------------------------------------------
def _make_slot_cutter(x, z, span, width, thickness_mm):
    """Create an X-oriented stadium slot cutter through the plate."""
    r = width / 2.0
    y0 = -thickness_mm / 2.0 - 2.0
    length_y = thickness_mm + 4.0

    c1 = Part.makeCylinder(r, length_y, App.Vector(x - span / 2.0, y0, z), App.Vector(0, 1, 0))
    c2 = Part.makeCylinder(r, length_y, App.Vector(x + span / 2.0, y0, z), App.Vector(0, 1, 0))
    box = Part.makeBox(span, length_y, width)
    box.translate(App.Vector(x - span / 2.0, y0, z - width / 2.0))

    shape = c1.fuse(c2).fuse(box)
    return shape


def _make_round_cutter(x, z, d, thickness_mm):
    r = d / 2.0
    y0 = -thickness_mm / 2.0 - 2.0
    length_y = thickness_mm + 4.0
    return Part.makeCylinder(r, length_y, App.Vector(x, y0, z), App.Vector(0, 1, 0))


def _make_corner_cutter(x, z, thickness_mm):
    """Create an M3 through-hole with a counterbore and support-free chamfer."""
    through_hole = _make_round_cutter(x, z, D_CORNER, thickness_mm)
    if CORNER_CHAMFER_DEPTH > CORNER_HEAD_DEPTH:
        raise ValueError("CORNER_CHAMFER_DEPTH cannot exceed CORNER_HEAD_DEPTH")

    recess_bottom_y = thickness_mm / 2.0 - CORNER_HEAD_DEPTH
    straight_recess = Part.makeCylinder(
        CORNER_COUNTERBORE_DIAMETER / 2.0,
        CORNER_HEAD_DEPTH - CORNER_CHAMFER_DEPTH + 0.1,
        App.Vector(x, recess_bottom_y + CORNER_CHAMFER_DEPTH, z),
        App.Vector(0, 1, 0))
    chamfer = Part.makeCone(
        D_CORNER / 2.0,
        CORNER_COUNTERBORE_DIAMETER / 2.0,
        CORNER_CHAMFER_DEPTH,
        App.Vector(x, recess_bottom_y, z),
        App.Vector(0, 1, 0))
    return through_hole.fuse(straight_recess).fuse(chamfer)


def _make_ribs(real_width, height_mm, thickness_mm):
    """Create full-width rear ribs on the negative-Y side of the plate."""
    ribs = []
    z_center = (N_ROWS - 1) * PITCH / 2.0
    ri = RIB_ROW_START
    while True:
        z = ri * PITCH - z_center
        if z > height_mm / 2.0:
            break
        if -height_mm / 2.0 <= z <= height_mm / 2.0:
            rib = Part.makeBox(real_width, RIB_DEPTH_Y, RIB_WIDTH_Z)
            rib.translate(App.Vector(-real_width / 2.0,
                                      -thickness_mm / 2.0 - RIB_DEPTH_Y,
                                      z - RIB_WIDTH_Z / 2.0))
            ribs.append(rib)
        ri += RIB_EVERY_N_ROWS
    return ribs


def build_plate(doc, width_mm=None, n_cols=None, height_mm=HEIGHT_MM,
                 thickness_mm=THICKNESS_MM, add_ribs=ADD_RIBS, name=None):
    """Create one perforated plate in the FreeCAD document `doc`.

    Supply either a target width or an explicit grid column count.
    """
    if n_cols is None:
        if width_mm is None:
            raise ValueError("Provide width_mm or n_cols")
        n_cols, real_width = columns_for_width(width_mm)
    else:
        real_width = (n_cols - 1) * PITCH + 2 * MARGIN_X

    if name is None:
        name = "Plate_%dmm" % round(real_width)

    # Solid plate centred in X/Z, with thickness in Y.
    box = Part.makeBox(real_width, thickness_mm, height_mm)
    box.translate(App.Vector(-real_width / 2.0, -thickness_mm / 2.0, -height_mm / 2.0))

    # Rear ribs are fused before the holes are cut.
    if add_ribs:
        for rib in _make_ribs(real_width, height_mm, thickness_mm):
            box = box.fuse(rib)

    # Round holes and slots. Slots that would overflow are replaced by rounds.
    grid = build_hole_grid(n_cols, plate_width=real_width)

    cutters = []
    for h in grid:
        if h['kind'] == 'slot':
            cutters.append(_make_slot_cutter(h['x'], h['z'], h['span'], h['width'], thickness_mm))
        elif h['kind'] == 'corner':
            cutters.append(_make_corner_cutter(h['x'], h['z'], thickness_mm))
        else:
            cutters.append(_make_round_cutter(h['x'], h['z'], h['d'], thickness_mm))

    if cutters:
        tool = cutters[0]
        for c in cutters[1:]:
            tool = tool.fuse(c)
        result = box.cut(tool)
    else:
        result = box
    result = result.removeSplitter()

    obj = doc.addObject("Part::Feature", name)
    obj.Shape = result
    obj.Label = name
    n_slots = sum(1 for h in grid if h['kind'] == 'slot')
    n_round = sum(1 for h in grid if h['kind'] == 'round')
    n_corner = sum(1 for h in grid if h['kind'] == 'corner')
    print("-> %s: %d columns x %d rows, actual width = %.3f mm "
            "(%d round holes, %d counterbored M3 holes, %d slots)"
          % (name, n_cols, N_ROWS, real_width, n_round, n_corner, n_slots))
    return obj


# --------------------------------------------------------------------------
# USAGE: set the desired width, then run the macro in FreeCAD.
# --------------------------------------------------------------------------
#TARGET_WIDTH_MM = 232.827
TARGET_WIDTH_MM = 160

if __name__ == "__main__":
    doc = App.ActiveDocument
    if doc is None:
        doc = App.newDocument("PerforatedPlates")

    build_plate(doc, width_mm=TARGET_WIDTH_MM)

    doc.recompute()

    App.Gui.ActiveDocument.ActiveView.viewIsometric() if hasattr(App, "Gui") else None