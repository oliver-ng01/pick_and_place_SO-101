"""
Find the centre of the green box's footprint on the table from its bottom edges.

Only the box's bottom edges touch the table, so they're the only part the table homography
maps correctly. Map them to the table, fit a straight line to each visible side, then use the
box's known size to place the centre:
  two sides visible -> extend the lines to the sharp corner, step half the length and half the width
  one side visible  -> step half the other dimension in from the side's midpoint

Before importing numpy anywhere, the calling script must set MKL_THREADING_LAYER=SEQUENTIAL (see main.py).
"""

import cv2
import numpy as np

BOX_LENGTH = 0.0515  # m, sharp-corner dimensions of the footprint
BOX_WIDTH = 0.035
GREEN_LOW = np.array([36, 50, 30])  # brightness min 30, so the box still counts when it's in shadow
GREEN_HIGH = np.array([89, 255, 255])


def to_table(H, pixels):
    return cv2.perspectiveTransform(np.asarray(pixels, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)


def to_pixels(H, table_points):
    return to_table(np.linalg.inv(H), table_points)


def find_box_contour(frame):
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), GREEN_LOW, GREEN_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)


def bottom_edge_pieces(contour):
    """Straight pieces of the outline's lower side, as (start, end) pixels."""
    hull = cv2.convexHull(contour.astype(np.float32))
    poly = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True).reshape(-1, 2).astype(np.float64)
    if np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]) < 0:
        poly = poly[::-1]  # make the order counter-clockwise in image coordinates
    pieces = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        d = (b - a) / np.linalg.norm(b - a)
        normal = np.array([d[1], -d[0]])  # points out of the shape for this vertex order
        if normal[1] > 0.6:  # faces down in the image: lower side, roughly sideways
            pieces.append((a, b))
    return pieces


def outline_points_on(contour, a, b, max_dist=2.0):
    """Outline pixels lying along segment a-b, skipping its last 15% at each end (the rounded corners)."""
    d = b - a
    t = (contour - a) @ d / (d @ d)
    dist = np.abs((contour[:, 0] - a[0]) * d[1] - (contour[:, 1] - a[1]) * d[0]) / np.linalg.norm(d)
    return contour[(dist < max_dist) & (t > 0.15) & (t < 0.85)]


def fit_side(points_table):
    vx, vy, x0, y0 = cv2.fitLine(points_table.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    d = np.array([vx, vy], np.float64)
    p = np.array([x0, y0], np.float64)
    t = (points_table - p) @ d
    return p, d, t.min(), t.max()


def box_center(frame, H):
    """Returns (centre_xy, footprint_corners_xy, sides_used) in table metres, or None if no box is found."""
    contour = find_box_contour(frame)
    if contour is None:
        return None
    pieces = bottom_edge_pieces(contour)
    if not pieces:
        return None

    # Map each piece to the table, then merge pieces pointing the same way (one side split by a rounded corner).
    sides = []  # list of table-point arrays, one per side
    for a, b in pieces:
        on_edge = outline_points_on(contour, a, b)
        if len(on_edge) < 3:  # too few pixels to fit a line (check before converting: OpenCV returns None for no points)
            continue
        pts = to_table(H, on_edge)
        _, d, _, _ = fit_side(pts)
        for i, side in enumerate(sides):
            if abs(fit_side(side)[1] @ d) > np.cos(np.radians(25)):
                sides[i] = np.vstack([side, pts])
                break
        else:
            sides.append(pts)
    if not sides:
        return None
    sides.sort(key=len, reverse=True)
    fits = [fit_side(s) for s in sides[:2]]

    inside = to_table(H, contour.mean(axis=0, keepdims=True))[0]  # the blob's mapped centre is always on the box's far side

    if len(fits) == 2 and abs(fits[0][1] @ fits[1][1]) < np.cos(np.radians(60)):
        (p1, d1, a1, b1), (p2, d2, a2, b2) = fits
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        corner = p1 + d1 * (((p2 - p1)[0] * d2[1] - (p2 - p1)[1] * d2[0]) / cross)
        u1 = d1 if (p1 + d1 * (a1 + b1) / 2 - corner) @ d1 > 0 else -d1  # along side 1, away from the corner
        u2 = d2 if (p2 + d2 * (a2 + b2) / 2 - corner) @ d2 > 0 else -d2
        if (b1 - a1) >= (b2 - a2):
            u_long, u_short = u1, u2
        else:
            u_long, u_short = u2, u1
        centre = corner + u_long * BOX_LENGTH / 2 + u_short * BOX_WIDTH / 2
        used = 2
    else:
        p, d, a, b = fits[0]
        mid = p + d * (a + b) / 2
        is_long = (b - a) > (BOX_LENGTH + BOX_WIDTH) / 2
        n = np.array([-d[1], d[0]])
        if (inside - mid) @ n < 0:
            n = -n  # step into the box, not away from it
        u_long, u_short = (d, n) if is_long else (n, d)
        centre = mid + n * ((BOX_WIDTH if is_long else BOX_LENGTH) / 2)
        used = 1

    half_l, half_w = u_long * BOX_LENGTH / 2, u_short * BOX_WIDTH / 2
    corners = np.array([centre - half_l - half_w, centre + half_l - half_w, centre + half_l + half_w, centre - half_l + half_w])
    return centre, corners, used
