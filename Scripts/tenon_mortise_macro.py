# -*- coding: utf-8 -*-
"""
FreeCAD macro: generates a grid of STRAIGHT tenons/mortises (no dovetail,
no draft angle) across a flat parting face, for 3D-printed enclosures
split into multiple parts.

Key feature: the two parts only need to be MANUALLY aligned (no shared
cutting tool / boolean history required). The macro automatically:
    - finds the matching face on part_b (opposite normal, closest plane)
    - measures the real offset/gap between the two planes
    - uses that offset to position the mortise correctly in part_b
    - adds a configurable safety margin to absorb alignment imprecision,
      on top of the printing clearance used for the fit itself

Usage:
    1. Open your document with both parts already positioned (manually
       aligned, roughly face to face).
    2. In the 3D view, click the flat parting face on part_a (the part
       that will receive the tenons).
    3. Set PART_B_NAME below to the internal Name of the other part.
    4. Adjust the parameters in the PARAMETERS section and run the macro.
"""

import FreeCAD as App
import Part


def find_matching_face(face_a, part_b, max_search_offset=8.0,
                        normal_alignment_tol=0.9):
    """
    Finds the planar face on part_b that faces face_a (near-opposite
    normal) and is closest to it along the normal axis.

    Returns (face_b, offset) where offset is the signed distance (mm)
    from face_a's plane to face_b's plane, measured along face_a's
    outward normal. offset ~ 0 means the faces are touching;
    offset > 0 means there is a gap; offset < 0 means slight overlap.

    Raises ValueError if no suitable face is found within
    max_search_offset.
    """
    normal = face_a.normalAt(0, 0).normalize()
    origin = face_a.CenterOfMass

    best_face = None
    best_offset = None

    for f in part_b.Shape.Faces:
        if not isinstance(f.Surface, Part.Plane):
            continue
        f_normal = f.normalAt(0, 0).normalize()
        if normal.dot(f_normal) > -normal_alignment_tol:
            continue  # not facing the opposite direction, skip

        f_center = f.CenterOfMass
        offset = (f_center - origin).dot(normal)

        if abs(offset) > max_search_offset:
            continue

        if best_offset is None or abs(offset) < abs(best_offset):
            best_face = f
            best_offset = offset

    if best_face is None:
        raise ValueError(
            "No matching face found on part_b within %.2f mm of face_a. "
            "Check alignment, or increase max_search_offset." % max_search_offset
        )

    return best_face, best_offset


def make_oriented_box(base_point, dir_u, dir_v, dir_n, len_u, len_v, len_n):
    """Builds a Part.Box oriented along three arbitrary orthonormal vectors."""
    box = Part.makeBox(len_u, len_v, len_n)
    mat = App.Matrix(
        dir_u.x, dir_v.x, dir_n.x, base_point.x,
        dir_u.y, dir_v.y, dir_n.y, base_point.y,
        dir_u.z, dir_v.z, dir_n.z, base_point.z,
        0, 0, 0, 1
    )
    return box.transformGeometry(mat)


def tenon_grid_on_face(part_a, part_b, face_a,
                        peg_size=6.0, pitch=15.0, margin=4.0,
                        depth=2.5, clearance=0.2, safety_margin=0.5,
                        chamfer=0.3, max_search_offset=8.0):
    """
    Generates a grid of straight tenons/mortises across a flat parting face.

    part_a, part_b : FreeCAD objects (Part::Feature)
                      -> part_a receives the tenons (material added)
                      -> part_b receives the mortises (material removed)
    face_a          : Part.Face, planar, belonging to part_a's shape,
                       representing the parting surface
    peg_size        : side length of each square peg (mm)
    pitch           : center-to-center spacing between pegs (mm)
    margin          : minimum distance between a peg and the face
                       boundary (mm)
    depth           : desired engagement depth of the tenon INTO part_b's
                       material (mm) -- does not include the gap, that is
                       handled automatically via the detected offset
    clearance       : printing fit clearance added to the mortise on
                       each side, for the peg to actually slide in (mm)
    safety_margin   : extra allowance added to the mortise, independent
                       from the printing clearance, to absorb imprecision
                       in the manual alignment between the two faces (mm).
                       Applied both laterally and in depth (before and
                       after the real face_b plane).
    chamfer         : chamfer on tenon edges to ease assembly (mm).
                       Use 0 to disable.
    max_search_offset : maximum expected gap between the two faces (mm),
                       used both to find the matching face on part_b and
                       as a sanity check on the alignment.

    Returns the number of pegs generated.
    """
    doc = App.ActiveDocument

    if not isinstance(face_a.Surface, Part.Plane):
        raise ValueError("The provided face is not planar.")

    normal = face_a.normalAt(0, 0).normalize()

    # Build a local in-plane basis (u_dir, v_dir)
    u_dir = normal.cross(App.Vector(1, 0, 0))
    if u_dir.Length < 1e-6:
        u_dir = normal.cross(App.Vector(0, 1, 0))
    u_dir = u_dir.normalize()
    v_dir = normal.cross(u_dir).normalize()

    origin = face_a.CenterOfMass

    # Find the real matching face on part_b and the actual gap/offset
    face_b, offset = find_matching_face(face_a, part_b, max_search_offset)
    App.Console.PrintMessage(
        "Matching face found on part_b, measured offset = %.3f mm\n" % offset
    )
    if abs(offset) > max_search_offset * 0.8:
        App.Console.PrintWarning(
            "Offset between faces (%.3f mm) is close to the search limit "
            "(%.3f mm) -- double check alignment.\n" % (offset, max_search_offset)
        )

    bbox = face_a.BoundBox
    half_extent = (bbox.XLength ** 2 + bbox.YLength ** 2 + bbox.ZLength ** 2) ** 0.5 / 2.0
    n_steps = int(half_extent // pitch) + 2

    tenon_boxes = []
    mortise_boxes = []
    count = 0
    skipped_out_of_bounds_b = 0

    for i in range(-n_steps, n_steps + 1):
        for j in range(-n_steps, n_steps + 1):
            pt = origin + u_dir * (i * pitch) + v_dir * (j * pitch)

            try:
                inside_a = face_a.isInside(pt, 1e-3, True)
            except Exception:
                inside_a = face_a.BoundBox.isInside(pt)
            if not inside_a:
                continue

            half = peg_size / 2.0 + margin
            edge_ok = all(
                face_a.isInside(pt + off, 1e-3, True)
                for off in [u_dir * half, u_dir * -half,
                            v_dir * half, v_dir * -half]
            )
            if not edge_ok:
                continue

            # Check the corresponding point actually falls within face_b's
            # real boundary too (projected along the normal by the offset)
            pt_on_b = pt + normal * offset
            try:
                inside_b = face_b.isInside(pt_on_b, 1e-3, True)
            except Exception:
                inside_b = face_b.BoundBox.isInside(pt_on_b)
            if not inside_b:
                skipped_out_of_bounds_b += 1
                continue

            base = pt - u_dir * (peg_size / 2.0) - v_dir * (peg_size / 2.0)

            # --- Tenon: grows from face_a across the gap, engaging
            #     'depth' mm into part_b's material ---
            tenon_length = offset + depth
            box_tenon = make_oriented_box(base, u_dir, v_dir, normal,
                                           peg_size, peg_size, tenon_length)
            if chamfer > 0:
                try:
                    box_tenon = box_tenon.makeChamfer(chamfer, box_tenon.Edges)
                except Exception:
                    pass
            tenon_boxes.append(box_tenon)

            # --- Mortise: starts slightly before the real face_b plane
            #     and extends deeper, using clearance for fit and
            #     safety_margin for alignment tolerance ---
            lateral_grow = clearance + safety_margin
            base_m = (base
                      - u_dir * (lateral_grow / 2.0)
                      - v_dir * (lateral_grow / 2.0)
                      + normal * (offset - safety_margin))
            mortise_depth_len = depth + 2 * safety_margin
            box_mortise = make_oriented_box(
                base_m, u_dir, v_dir, normal,
                peg_size + lateral_grow, peg_size + lateral_grow,
                mortise_depth_len
            )
            mortise_boxes.append(box_mortise)
            count += 1

    if skipped_out_of_bounds_b > 0:
        App.Console.PrintWarning(
            "%d peg(s) skipped: position falls outside part_b's face "
            "boundary. Check alignment or reduce grid extent.\n"
            % skipped_out_of_bounds_b
        )

    if count == 0:
        App.Console.PrintWarning("No pegs generated -- check pitch/margin/face.\n")
        return 0

    tenons_compound = tenon_boxes[0]
    for b in tenon_boxes[1:]:
        tenons_compound = tenons_compound.fuse(b)

    mortises_compound = mortise_boxes[0]
    for b in mortise_boxes[1:]:
        mortises_compound = mortises_compound.fuse(b)

    part_a.Shape = part_a.Shape.fuse(tenons_compound).removeSplitter()
    part_b.Shape = part_b.Shape.cut(mortises_compound).removeSplitter()

    doc.recompute()
    return count


# =========================================================================
# PARAMETERS
# =========================================================================
PART_B_NAME = "Enclosure_PartB"   # internal Name of the mating part

if __name__ == "__main__":
    import FreeCADGui as Gui

    sel = Gui.Selection.getSelectionEx()

    if sel and sel[0].SubObjects and hasattr(sel[0].SubObjects[0], "Surface"):
        part_a_obj = sel[0].Object
        face_a_sel = sel[0].SubObjects[0]

        doc = App.ActiveDocument
        part_b_obj = doc.getObject(PART_B_NAME)

        if part_b_obj is None:
            App.Console.PrintError(
                "Set PART_B_NAME to the internal Name of the mating part.\n"
            )
        else:
            try:
                n = tenon_grid_on_face(
                    part_a_obj, part_b_obj, face_a_sel,
                    peg_size=6.0,          # peg side (mm)
                    pitch=15.0,            # spacing between pegs (mm)
                    margin=4.0,            # margin from face boundary (mm)
                    depth=2.5,             # engagement depth into part_b (mm)
                    clearance=0.2,         # printing fit clearance (mm)
                    safety_margin=0.5,     # alignment tolerance margin (mm)
                    chamfer=0.3,
                    max_search_offset=8.0  # expected max gap between faces (mm)
                )
                App.Console.PrintMessage("%d tenon/mortise pegs generated.\n" % n)
            except ValueError as e:
                App.Console.PrintError(str(e) + "\n")
    else:
        App.Console.PrintError(
            "Select a planar parting face in the 3D view before running "
            "the macro.\n"
        )
