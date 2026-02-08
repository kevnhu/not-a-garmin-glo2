/*
 * ForeFlight WiFi GPS Enclosure
 *
 * Two-part case (base + lid) joined with 4x M3 bolts.
 * M3 bolts pass through lid (3.3mm) and self-thread into base (2.9mm).
 * Solid corner gussets merge bolt bosses with walls for maximum strength.
 *
 * Components:
 *   ESP32 (55x30mm) - double-sided tape to base floor
 *   M8N GPS (25x25mm) - glued into lid lip, antenna pokes through 20x20 hole
 *   OLED (25x28mm) - mounted in lid rotated 90 degrees, visible through window
 *
 * Print both parts open-side-up. Flip lid when assembling.
 * Material: PETG recommended (cockpit heat resistance)
 * Print settings: 0.2mm layers, 3 perimeters, 20% infill
 */

// ===================== PARAMETERS =====================

/* [Shell] */
wall     = 2;        // Wall/floor/ceiling thickness (mm)
round_r  = 3;        // Exterior corner rounding radius
chamfer  = 1.5;      // 45-degree chamfer on outer edges

/* [Interior Cavity] */
cav_l    = 70;       // Interior length (X)
cav_w    = 44;       // Interior width (Y)
base_d   = 12;       // Base interior depth
lid_d    = 8;        // Lid interior depth

/* [ESP32 Board] */
esp_l    = 55;       // Board length
esp_w    = 30;       // Board width

/* [M8N GPS Module] */
gps_s    = 25;       // Module size (25x25)
gps_hole = 20;       // Antenna opening (20x20)

/* [OLED Display - rotated 90 degrees] */
oled_pcb_l  = 28;    // PCB length (before rotation)
oled_pcb_w  = 25;    // PCB width (before rotation)
oled_l   = 25;       // Module extent along X (after rotation)
oled_w   = 28;       // Module extent along Y (after rotation)
oled_vis_l = 13;     // Visible screen area along X
oled_vis_w = 25;     // Visible screen area along Y

/* [M3 Fasteners] */
thd_d    = 2.9;      // Self-threading hole diameter (base only)
clr_d    = 3.3;      // Clearance through-hole diameter (lid only)
boss_d   = 7;        // Boss outer diameter
boss_in  = 2;        // Boss center inset from interior wall
fillet_r = 3;        // Inner corner fillet radius

/* [USB Port Cutout] */
usb_w    = 12;       // Port width
usb_h    = 7;        // Port height

/* [Vertical Ventilation Slots] */
vent_n   = 10;       // Number of slots per side
vent_w   = 2;        // Slot width (along wall length)
vent_gap = 2.5;      // Gap between slots
vent_top = 3;        // Solid material above vents (enclosed top)

/* [Resolution] */
$fn      = 48;

// ===================== DERIVED VALUES =====================

ext_l    = cav_l + 2 * wall;
ext_w    = cav_w + 2 * wall;
base_h   = base_d + wall;
lid_h    = lid_d + wall;
gps_lip  = (gps_s - gps_hole) / 2;

// Bolt positions (exterior coordinates)
bolt_edge = wall + boss_in;
bolts = [
    [bolt_edge,         bolt_edge],
    [ext_l - bolt_edge, bolt_edge],
    [bolt_edge,         ext_w - bolt_edge],
    [ext_l - bolt_edge, ext_w - bolt_edge]
];

// Corner gusset size (from edge to far side of boss)
gusset = bolt_edge + boss_d / 2;

// Component positions (interior coordinates)
oled_x = (cav_l / 2 - oled_l) / 2;
oled_y = (cav_w - oled_w) / 2;
gps_x  = cav_l / 2 + (cav_l / 2 - gps_s) / 2;
gps_y  = (cav_w - gps_s) / 2;

// Vent slot layout (centered on long walls)
vent_total = vent_n * vent_w + (vent_n - 1) * vent_gap;
vent_start = (ext_l - vent_total) / 2;
vent_h     = base_d - vent_top;  // Slot height (floor to top minus margin)

// ===================== MODULES =====================

// Rounded rectangle extrusion
module rrect(l, w, h, r) {
    r2 = min(r, min(l, w) / 2 - 0.01);
    hull() {
        translate([r2,     r2,     0]) cylinder(r = r2, h = h);
        translate([l - r2, r2,     0]) cylinder(r = r2, h = h);
        translate([r2,     w - r2, 0]) cylinder(r = r2, h = h);
        translate([l - r2, w - r2, 0]) cylinder(r = r2, h = h);
    }
}

// 45-degree chamfer subtracted from outer edge (true taper via hull)
module chamfer_edge(l, w, r, ch, z_pos) {
    translate([0, 0, z_pos])
        difference() {
            translate([-1, -1, -0.01])
                cube([l + 2, w + 2, ch + 0.02]);
            hull() {
                // Smaller footprint at z=0 (the exposed edge)
                translate([ch, ch, 0])
                    rrect(l - 2 * ch, w - 2 * ch, 0.01, max(0.5, r - ch));
                // Full footprint at z=ch (merges flush with body)
                translate([0, 0, ch])
                    rrect(l, w, 0.01, r);
            }
        }
}

// Fillet cutters: round the sharp inner edge of each corner gusset
// For each corner, subtracts a (square minus quarter-circle) wedge
module gusset_fillets(r, h) {
    fh = h - wall;  // only cut cavity depth, preserve floor/ceiling
    for (b = bolts) {
        ix = (b[0] < ext_l / 2) ? gusset : ext_l - gusset;
        iy = (b[1] < ext_w / 2) ? gusset : ext_w - gusset;
        dx = (b[0] < ext_l / 2) ? -1 : 1;
        dy = (b[1] < ext_w / 2) ? -1 : 1;
        translate([ix + min(0, dx) * r, iy + min(0, dy) * r, wall])
            difference() {
                cube([r, r, fh + 0.1]);
                translate([(dx > 0) ? r : 0, (dy > 0) ? r : 0, -0.1])
                    cylinder(r = r, h = fh + 0.3);
            }
    }
}

// ===================== BASE =====================
// Bosses are added AFTER the cavity is cut, so they stay solid.
// Only the bolt hole (2.9mm) is drilled through them.

module base() {
    inner_r = max(1, round_r - wall);

    difference() {
        union() {
            // Step 1: Shell with cavity already removed
            difference() {
                rrect(ext_l, ext_w, base_h, round_r);

                translate([wall, wall, wall])
                    rrect(cav_l, cav_w, base_d + 1, inner_r);
            }

            // Step 2: Solid corner gussets (boss merged with walls)
            // Added AFTER cavity cut so they remain fully solid
            for (b = bolts) {
                gx = (b[0] < ext_l / 2) ? 0 : ext_l - gusset;
                gy = (b[1] < ext_w / 2) ? 0 : ext_w - gusset;
                intersection() {
                    rrect(ext_l, ext_w, base_h, round_r);
                    translate([gx, gy, 0])
                        cube([gusset, gusset, base_h]);
                }
            }

        }

        // M3 self-threading holes (2.9mm) through solid gussets
        for (b = bolts)
            translate([b[0], b[1], -0.1])
                cylinder(d = thd_d, h = base_h + 0.2);

        // USB port cutout (left wall, centered on Y)
        translate([-0.1, (ext_w - usb_w) / 2, wall + 1])
            cube([wall + 0.2, usb_w, usb_h]);

        // Vertical vent slots on FRONT wall (Y=0)
        // Run from floor (z=wall) up, enclosed at top with vent_top mm
        for (i = [0 : vent_n - 1])
            translate([
                vent_start + i * (vent_w + vent_gap),
                -0.1,
                wall
            ])
                cube([vent_w, wall + 0.2, vent_h]);

        // Vertical vent slots on BACK wall (Y=ext_w)
        for (i = [0 : vent_n - 1])
            translate([
                vent_start + i * (vent_w + vent_gap),
                ext_w - wall - 0.1,
                wall
            ])
                cube([vent_w, wall + 0.2, vent_h]);

        // Round inner corners of gussets (3mm fillet)
        gusset_fillets(fillet_r, base_h);

        // 45-degree chamfer on bottom outer edge
        chamfer_edge(ext_l, ext_w, round_r, chamfer, -0.01);
    }
}

// ===================== LID =====================
// Bosses added after cavity cut to stay solid.
// 3.3mm clearance holes only.

module lid() {
    inner_r = max(1, round_r - wall);
    shelf_h = 1.5;
    oled_lip = 1.5;

    difference() {
        union() {
            // Step 1: Shell with cavity removed
            difference() {
                rrect(ext_l, ext_w, lid_h, round_r);

                translate([wall, wall, wall])
                    rrect(cav_l, cav_w, lid_d + 1, inner_r);
            }

            // Step 2: Solid corner gussets (boss merged with walls)
            for (b = bolts) {
                gx = (b[0] < ext_l / 2) ? 0 : ext_l - gusset;
                gy = (b[1] < ext_w / 2) ? 0 : ext_w - gusset;
                intersection() {
                    rrect(ext_l, ext_w, lid_h, round_r);
                    translate([gx, gy, 0])
                        cube([gusset, gusset, lid_h]);
                }
            }

        }

        // M3 clearance through-holes (3.3mm)
        for (b = bolts)
            translate([b[0], b[1], -0.1])
                cylinder(d = clr_d, h = lid_h + 0.2);

        // GPS antenna opening: 20x20mm through ceiling
        translate([
            wall + gps_x + gps_lip,
            wall + gps_y + gps_lip,
            -0.1
        ])
            cube([gps_hole, gps_hole, wall + 0.2]);

        // OLED display window through ceiling (rotated 90°)
        translate([
            wall + oled_x + (oled_l - oled_vis_l) / 2,
            wall + oled_y + (oled_w - oled_vis_w) / 2,
            -0.1
        ])
            cube([oled_vis_l, oled_vis_w, wall + 0.2]);

        // Round inner corners of gussets (3mm fillet)
        gusset_fillets(fillet_r, lid_h);

        // 45-degree chamfer on exterior top edge
        chamfer_edge(ext_l, ext_w, round_r, chamfer, -0.01);
    }
}

// ===================== RENDER =====================

color("DimGray") base();

translate([ext_l + 15, 0, 0])
    color("SlateGray") lid();

// ===================== DIMENSIONS REFERENCE =====================
// Exterior: 74 x 48 x 14mm (base), 74 x 48 x 10mm (lid)
// Total assembled height: ~24mm
// Corner gussets: 8.5mm solid blocks in each corner, clipped to rounded shell
//   Base: 2.9mm threaded holes | Lid: 3.3mm clearance holes
// Chamfer: 1.5mm 45-degree on outer edges
// Vents: 10 vertical slots per side (front + back walls), spanning full length
//   Run from floor up, 3mm solid band at top
// GPS antenna: 20x20mm opening (right half of lid)
// OLED window: 13x25mm (left half of lid, rotated 90°)
// USB cutout: 12x7mm (left wall of base)
