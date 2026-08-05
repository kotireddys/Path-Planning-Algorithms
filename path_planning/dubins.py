import math
from typing import List, Tuple


def _arc_points(center_x, center_y, r, start_ang, end_ang, step_ang):
    pts = []
    # normalize
    if end_ang < start_ang:
        end_ang += 2 * math.pi
    a = start_ang
    while a <= end_ang + 1e-6:
        x = center_x + r * math.cos(a)
        y = center_y + r * math.sin(a)
        pts.append((x, y))
        a += step_ang
    return pts


def dubins_path(start: Tuple[float, float, float], goal: Tuple[float, float, float], turning_radius: float = 5.0, step_size: float = 0.5) -> List[Tuple[float, float]]:
    """
    Approximate Dubins path: a simple heuristic composed of three segments:
    - initial turn from start heading towards a tangent
    - straight-line segment
    - final turn to reach goal heading

    This is not a full optimal Dubins implementation but provides a usable
    continuous-curvature path for demos. Returns list of (x,y) points.
    """
    x0, y0, th0 = start
    x1, y1, th1 = goal

    # compute line connecting the two points
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return [(x0, y0)]

    # derive headings if None-like values provided
    # create initial and final arc centers (left-turn centers)
    # We'll build a simple path: turn toward goal direction, go straight, then turn to final yaw.
    ang_to_goal = math.atan2(dy, dx)

    # initial turn: from th0 to ang_to_goal
    delta0 = (ang_to_goal - th0)
    while delta0 <= -math.pi:
        delta0 += 2 * math.pi
    while delta0 > math.pi:
        delta0 -= 2 * math.pi

    # final turn: from ang_to_goal to th1
    delta1 = (th1 - ang_to_goal)
    while delta1 <= -math.pi:
        delta1 += 2 * math.pi
    while delta1 > math.pi:
        delta1 -= 2 * math.pi

    # generate initial arc
    step_ang = step_size / max(turning_radius, 1e-6)
    center0_x = x0 - turning_radius * math.sin(th0)
    center0_y = y0 + turning_radius * math.cos(th0)
    start_ang0 = math.atan2(y0 - center0_y, x0 - center0_x)
    end_ang0 = start_ang0 + delta0
    arc0 = _arc_points(center0_x, center0_y, turning_radius, start_ang0, end_ang0, step_ang)

    # straight segment from end of arc0 to start of final arc
    straight_start = arc0[-1]
    # compute final arc center from goal pose
    center1_x = x1 - turning_radius * math.sin(th1)
    center1_y = y1 + turning_radius * math.cos(th1)
    start_ang1 = math.atan2(straight_start[1] - center1_y, straight_start[0] - center1_x)
    end_ang1 = math.atan2(y1 - center1_y, x1 - center1_x)

    # straight length: project between tangent points (approx)
    straight = []
    # pick a straight end-point as the tangent start to final arc
    # here simply use point on line toward goal offset by turning radius
    dir_x = math.cos(ang_to_goal)
    dir_y = math.sin(ang_to_goal)
    straight_len = max(0.0, dist - 2 * turning_radius)
    n_steps = max(1, int(straight_len / step_size))
    for i in range(1, n_steps + 1):
        t = i / n_steps
        sx = straight_start[0] + dir_x * (straight_len * t)
        sy = straight_start[1] + dir_y * (straight_len * t)
        straight.append((sx, sy))

    # final arc from start_ang1 to end_ang1
    arc1 = _arc_points(center1_x, center1_y, turning_radius, start_ang1, end_ang1, step_ang)

    path = []
    path.extend(arc0)
    path.extend(straight)
    path.extend(arc1)
    return path
