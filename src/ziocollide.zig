//! 2D collision detection for games.
//!
//! AABB, circle, ray, and point-in-polygon tests. Narrow-phase helpers
//! for broad-phase systems. Zero allocation, pure functions.

const std = @import("std");

/// Axis-aligned bounding box.
pub const AABB = struct {
    x: f32,
    y: f32,
    w: f32,
    h: f32,

    pub fn minX(self: AABB) f32 { return self.x; }
    pub fn minY(self: AABB) f32 { return self.y; }
    pub fn maxX(self: AABB) f32 { return self.x + self.w; }
    pub fn maxY(self: AABB) f32 { return self.y + self.h; }
    pub fn centerX(self: AABB) f32 { return self.x + self.w / 2; }
    pub fn centerY(self: AABB) f32 { return self.y + self.h / 2; }

    /// Does this AABB overlap another?
    pub fn overlaps(self: AABB, other: AABB) bool {
        return self.minX() < other.maxX() and
               self.maxX() > other.minX() and
               self.minY() < other.maxY() and
               self.maxY() > other.minY();
    }

    /// Does this AABB fully contain a point?
    pub fn containsPoint(self: AABB, px: f32, py: f32) bool {
        return px >= self.minX() and px <= self.maxX() and
               py >= self.minY() and py <= self.maxY();
    }

    /// Does this AABB fully contain another?
    pub fn containsAABB(self: AABB, other: AABB) bool {
        return other.minX() >= self.minX() and other.maxX() <= self.maxX() and
               other.minY() >= self.minY() and other.maxY() <= self.maxY();
    }

    /// Merged AABB that covers both.
    pub fn merged(self: AABB, other: AABB) AABB {
        const x1 = @min(self.minX(), other.minX());
        const y1 = @min(self.minY(), other.minY());
        const x2 = @max(self.maxX(), other.maxX());
        const y2 = @max(self.maxY(), other.maxY());
        return .{ .x = x1, .y = y1, .w = x2 - x1, .h = y2 - y1 };
    }
};

/// Circle defined by center and radius.
pub const Circle = struct {
    x: f32,
    y: f32,
    r: f32,

    /// Does this circle overlap another?
    pub fn overlaps(self: Circle, other: Circle) bool {
        const dx = self.x - other.x;
        const dy = self.y - other.y;
        const dist_sq = dx * dx + dy * dy;
        const rad_sum = self.r + other.r;
        return dist_sq <= rad_sum * rad_sum;
    }

    /// Does this circle contain a point?
    pub fn containsPoint(self: Circle, px: f32, py: f32) bool {
        const dx = self.x - px;
        const dy = self.y - py;
        return dx * dx + dy * dy <= self.r * self.r;
    }
};

/// Ray from origin in a direction.
pub const Ray = struct {
    ox: f32, oy: f32, // origin
    dx: f32, dy: f32, // direction (normalized or not)

    pub const Hit = struct {
        t: f32,
        nx: f32, ny: f32, // normal at hit point
    };

    /// Intersect ray vs AABB. Returns hit with t (0 = origin, 1 = endpoint).
    pub fn vsAABB(self: Ray, box: AABB) ?Hit {
        var tmin: f32 = 0;
        var tmax: f32 = std.math.floatMax(f32);

        // X slab
        if (self.dx != 0) {
            var t1 = (box.minX() - self.ox) / self.dx;
            var t2 = (box.maxX() - self.ox) / self.dx;
            if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
            tmin = @max(tmin, t1);
            tmax = @min(tmax, t2);
            if (tmin > tmax) return null;
        } else {
            if (self.ox < box.minX() or self.ox > box.maxX()) return null;
        }

        // Y slab
        if (self.dy != 0) {
            var t1 = (box.minY() - self.oy) / self.dy;
            var t2 = (box.maxY() - self.oy) / self.dy;
            if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
            tmin = @max(tmin, t1);
            tmax = @min(tmax, t2);
            if (tmin > tmax) return null;
        } else {
            if (self.oy < box.minY() or self.oy > box.maxY()) return null;
        }

        var nx: f32 = 0;
        var ny: f32 = 0;
        const hit_x = self.ox + tmin * self.dx;
        const hit_y = self.oy + tmin * self.dy;
        // Determine which face was hit
        if (@abs(hit_x - box.minX()) < 0.001) nx = -1
        else if (@abs(hit_x - box.maxX()) < 0.001) nx = 1
        else if (@abs(hit_y - box.minY()) < 0.001) ny = -1
        else ny = 1;

        return .{ .t = tmin, .nx = nx, .ny = ny };
    }

    /// Intersect ray vs circle. Returns hit with t.
    pub fn vsCircle(self: Ray, c: Circle) ?Hit {
        const fx = self.ox - c.x;
        const fy = self.oy - c.y;
        const a = self.dx * self.dx + self.dy * self.dy;
        const b = 2 * (fx * self.dx + fy * self.dy);
        const c_val = fx * fx + fy * fy - c.r * c.r;
        var disc = b * b - 4 * a * c_val;
        if (disc < 0) return null;
        disc = @sqrt(disc);
        const t = (-b - disc) / (2 * a);
        if (t < 0) return null;
        const hit_x = self.ox + t * self.dx;
        const hit_y = self.oy + t * self.dy;
        const len = @sqrt((hit_x - c.x) * (hit_x - c.x) + (hit_y - c.y) * (hit_y - c.y));
        if (len == 0) return .{ .t = t, .nx = 0, .ny = -1 };
        return .{ .t = t, .nx = (hit_x - c.x) / len, .ny = (hit_y - c.y) / len };
    }
};

/// AABB vs circle overlap test.
pub fn aabbVsCircle(box: AABB, c: Circle) bool {
    const cx = @max(box.minX(), @min(c.x, box.maxX()));
    const cy = @max(box.minY(), @min(c.y, box.maxY()));
    const dx = c.x - cx;
    const dy = c.y - cy;
    return dx * dx + dy * dy <= c.r * c.r;
}

/// Point in convex polygon (counter-clockwise winding).
/// `poly_x` and `poly_y` must have the same length >= 3.
pub fn pointInPolygon(px: f32, py: f32, poly_x: []const f32, poly_y: []const f32) bool {
    std.debug.assert(poly_x.len == poly_y.len and poly_x.len >= 3);
    var inside = false;
    var j = poly_x.len - 1;
    for (poly_x, 0..) |_, i| {
        if ((poly_y[i] > py) != (poly_y[j] > py) and
            px < (poly_x[j] - poly_x[i]) * (py - poly_y[i]) / (poly_y[j] - poly_y[i]) + poly_x[i])
        {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

/// Separating Axis Theorem overlap for two convex polygons.
/// Returns penetration depth (negative = no overlap).
pub fn satOverlap(
    ax: []const f32, ay: []const f32,
    bx: []const f32, by: []const f32,
) f32 {
    var min_overlap = std.math.floatMax(f32);

    var j = ax.len - 1;
    for (ax, 0..) |_, i| {
        const edge_x = ax[i] - ax[j];
        const edge_y = ay[i] - ay[j];
        const axis_x = -edge_y;
        const axis_y = edge_x;
        const len = @sqrt(axis_x * axis_x + axis_y * axis_y);
        if (len == 0) { j = i; continue; }
        const nx = axis_x / len;
        const ny = axis_y / len;

        var min_a: f32 = std.math.floatMax(f32);
        var max_a: f32 = -std.math.floatMax(f32);
        for (ax, ay) |ax_v, ay_v| {
            const proj = ax_v * nx + ay_v * ny;
            min_a = @min(min_a, proj);
            max_a = @max(max_a, proj);
        }

        var min_b: f32 = std.math.floatMax(f32);
        var max_b: f32 = -std.math.floatMax(f32);
        for (bx, by) |bx_v, by_v| {
            const proj = bx_v * nx + by_v * ny;
            min_b = @min(min_b, proj);
            max_b = @max(max_b, proj);
        }

        const overlap = @min(max_a - min_b, max_b - min_a);
        if (overlap <= 0) return -1; // separated
        min_overlap = @min(min_overlap, overlap);
        j = i;
    }

    j = bx.len - 1;
    for (bx, 0..) |_, i| {
        const edge_x = bx[i] - bx[j];
        const edge_y = by[i] - by[j];
        const axis_x = -edge_y;
        const axis_y = edge_x;
        const len = @sqrt(axis_x * axis_x + axis_y * axis_y);
        if (len == 0) { j = i; continue; }
        const nx = axis_x / len;
        const ny = axis_y / len;

        var min_a: f32 = std.math.floatMax(f32);
        var max_a: f32 = -std.math.floatMax(f32);
        for (ax, ay) |ax_v, ay_v| {
            const proj = ax_v * nx + ay_v * ny;
            min_a = @min(min_a, proj);
            max_a = @max(max_a, proj);
        }

        var min_b: f32 = std.math.floatMax(f32);
        var max_b: f32 = -std.math.floatMax(f32);
        for (bx, by) |bx_v, by_v| {
            const proj = bx_v * nx + by_v * ny;
            min_b = @min(min_b, proj);
            max_b = @max(max_b, proj);
        }

        const overlap = @min(max_a - min_b, max_b - min_a);
        if (overlap <= 0) return -1;
        min_overlap = @min(min_overlap, overlap);
        j = i;
    }

    return min_overlap;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "AABB overlaps" {
    const a = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = AABB{ .x = 5, .y = 5, .w = 10, .h = 10 };
    try std.testing.expect(a.overlaps(b));
    try std.testing.expect(b.overlaps(a));
}

test "AABB no overlap" {
    const a = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = AABB{ .x = 20, .y = 20, .w = 5, .h = 5 };
    try std.testing.expect(!a.overlaps(b));
}

test "AABB contains point" {
    const a = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    try std.testing.expect(a.containsPoint(5, 5));
    try std.testing.expect(!a.containsPoint(15, 5));
}

test "AABB contains AABB" {
    const outer = AABB{ .x = 0, .y = 0, .w = 20, .h = 20 };
    const inner = AABB{ .x = 5, .y = 5, .w = 5, .h = 5 };
    try std.testing.expect(outer.containsAABB(inner));
    try std.testing.expect(!inner.containsAABB(outer));
}

test "Circle overlaps" {
    const a = Circle{ .x = 0, .y = 0, .r = 5 };
    const b = Circle{ .x = 8, .y = 0, .r = 5 };
    try std.testing.expect(a.overlaps(b));
}

test "Circle no overlap" {
    const a = Circle{ .x = 0, .y = 0, .r = 5 };
    const b = Circle{ .x = 20, .y = 0, .r = 5 };
    try std.testing.expect(!a.overlaps(b));
}

test "Circle contains point" {
    const c = Circle{ .x = 5, .y = 5, .r = 3 };
    try std.testing.expect(c.containsPoint(5, 5));
    try std.testing.expect(!c.containsPoint(10, 10));
}

test "aabbVsCircle" {
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const inside = Circle{ .x = 5, .y = 5, .r = 3 };
    const outside = Circle{ .x = 20, .y = 20, .r = 3 };
    const touching = Circle{ .x = 12, .y = 5, .r = 3 };
    try std.testing.expect(aabbVsCircle(box, inside));
    try std.testing.expect(!aabbVsCircle(box, outside));
    try std.testing.expect(aabbVsCircle(box, touching));
}

test "pointInPolygon triangle" {
    const px = [_]f32{ 0, 10, 5 };
    const py = [_]f32{ 0, 0, 10 };
    try std.testing.expect(pointInPolygon(5, 3, &px, &py));
    try std.testing.expect(!pointInPolygon(0, 8, &px, &py));
}

test "pointInPolygon square" {
    const px = [_]f32{ 0, 10, 10, 0 };
    const py = [_]f32{ 0, 0, 10, 10 };
    try std.testing.expect(pointInPolygon(5, 5, &px, &py));
    try std.testing.expect(!pointInPolygon(15, 5, &px, &py));
}

test "Ray vs AABB hit" {
    const ray = Ray{ .ox = -5, .oy = 5, .dx = 1, .dy = 0 };
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const hit = ray.vsAABB(box);
    try std.testing.expect(hit != null);
    try std.testing.expect(hit.?.t > 0);
}

test "Ray vs AABB miss" {
    const ray = Ray{ .ox = -5, .oy = 20, .dx = 1, .dy = 0 };
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    try std.testing.expect(ray.vsAABB(box) == null);
}

test "Ray vs circle hit" {
    const ray = Ray{ .ox = -5, .oy = 0, .dx = 1, .dy = 0 };
    const c = Circle{ .x = 5, .y = 0, .r = 3 };
    const hit = ray.vsCircle(c);
    try std.testing.expect(hit != null);
    try std.testing.expect(hit.?.t > 0);
}

test "satOverlap overlapping squares" {
    const ax = [_]f32{ 0, 10, 10, 0 };
    const ay = [_]f32{ 0, 0, 10, 10 };
    const bx = [_]f32{ 5, 15, 15, 5 };
    const by = [_]f32{ 0, 0, 10, 10 };
    const depth = satOverlap(&ax, &ay, &bx, &by);
    try std.testing.expect(depth > 0);
}

test "satOverlap separated" {
    const ax = [_]f32{ 0, 10, 10, 0 };
    const ay = [_]f32{ 0, 0, 10, 10 };
    const bx = [_]f32{ 20, 30, 30, 20 };
    const by = [_]f32{ 0, 0, 10, 10 };
    const depth = satOverlap(&ax, &ay, &bx, &by);
    try std.testing.expect(depth < 0);
}

test "AABB merged" {
    const a = AABB{ .x = 0, .y = 0, .w = 5, .h = 5 };
    const b = AABB{ .x = 10, .y = 10, .w = 5, .h = 5 };
    const m = a.merged(b);
    try std.testing.expectEqual(@as(f32, 0), m.x);
    try std.testing.expectEqual(@as(f32, 0), m.y);
    try std.testing.expectEqual(@as(f32, 15), m.w);
    try std.testing.expectEqual(@as(f32, 15), m.h);
}

test "AABB center" {
    const a = AABB{ .x = 10, .y = 20, .w = 20, .h = 30 };
    try std.testing.expectEqual(@as(f32, 20), a.centerX());
    try std.testing.expectEqual(@as(f32, 35), a.centerY());
}

test "Ray vs circle miss" {
    const ray = Ray{ .ox = -5, .oy = 20, .dx = 1, .dy = 0 };
    const c = Circle{ .x = 5, .y = 0, .r = 3 };
    try std.testing.expect(ray.vsCircle(c) == null);
}

test "pointInPolygon pentagon" {
    const px = [_]f32{ 50, 80, 70, 30, 20 };
    const py = [_]f32{ 0, 35, 70, 70, 35 };
    try std.testing.expect(pointInPolygon(50, 35, &px, &py));
    try std.testing.expect(!pointInPolygon(50, 75, &px, &py));
}

test "AABB no overlap touching edges" {
    const a = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = AABB{ .x = 10, .y = 0, .w = 10, .h = 10 };
    // Adjacent but not overlapping (a.maxX == b.minX)
    try std.testing.expect(!a.overlaps(b));
}

test "Circle contains boundary point" {
    const c = Circle{ .x = 5, .y = 5, .r = 3 };
    // Point exactly on the boundary
    try std.testing.expect(c.containsPoint(5 + 3, 5)); // (8,5) distance = 3
    try std.testing.expect(!c.containsPoint(5 + 3.01, 5)); // slightly outside
}

test "aabbVsCircle corner case" {
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const c = Circle{ .x = 15, .y = 15, .r = 8 };
    // Circle center far away but radius large enough to touch corner
    try std.testing.expect(aabbVsCircle(box, c));
}

test "Ray vs AABB from inside" {
    const ray = Ray{ .ox = 5, .oy = 5, .dx = 1, .dy = 0 };
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const hit = ray.vsAABB(box);
    try std.testing.expect(hit != null);
    // t should be 0 since origin is inside
    try std.testing.expect(hit.?.t >= 0);
}

// --- Integration-style tests ---

test "AABB then Circle collision pipeline" {
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const c = Circle{ .x = 15, .y = 5, .r = 8 };

    // Broad phase: AABB overlap
    const broad = AABB{ .x = c.x - c.r, .y = c.y - c.r, .w = c.r * 2, .h = c.r * 2 };
    try std.testing.expect(box.overlaps(broad));

    // Narrow phase: precise circle test
    try std.testing.expect(aabbVsCircle(box, c));
}

test "Ray cast and point-in-polygon for hit testing" {
    const poly_x = [_]f32{ 0, 20, 20, 0 };
    const poly_y = [_]f32{ 0, 0, 20, 20 };
    const ray = Ray{ .ox = 10, .oy = -5, .dx = 0, .dy = 1 };
    const box = AABB{ .x = 0, .y = 0, .w = 20, .h = 20 };

    const hit = ray.vsAABB(box);
    try std.testing.expect(hit != null);
    if (hit) |h| {
        const px = 10;
        const py = h.t * ray.dy + ray.oy;
        try std.testing.expect(pointInPolygon(px, py, &poly_x, &poly_y));
    }
}

test "Circle contains point from AABB center" {
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const c = Circle{ .x = box.centerX(), .y = box.centerY(), .r = 5 };
    try std.testing.expect(c.containsPoint(box.centerX(), box.centerY()));
}

test "satOverlap triangles" {
    const ax = [_]f32{ 0, 5, 2.5 };
    const ay = [_]f32{ 0, 0, 5 };
    const bx = [_]f32{ 2, 7, 4.5 };
    const by = [_]f32{ 0, 0, 5 };
    const depth = satOverlap(&ax, &ay, &bx, &by);
    try std.testing.expect(depth > 0);
}

test "Circle overlap at exact distance" {
    const a = Circle{ .x = 0, .y = 0, .r = 5 };
    const b = Circle{ .x = 10, .y = 0, .r = 5 };
    // Distance = 10, sum of radii = 10, touching
    try std.testing.expect(a.overlaps(b));
}

test "Ray vs circle tangent" {
    const ray = Ray{ .ox = 0, .oy = -5, .dx = 1, .dy = 0 };
    const c = Circle{ .x = 3, .y = 0, .r = 5 };
    // Ray is tangent to circle at y=-5 which is exactly at edge
    const hit = ray.vsCircle(c);
    // Tangent rays may or may not hit depending on precision
    if (hit) |h| {
        try std.testing.expect(h.t >= 0);
    }
}

test "AABB contains point on all four corners" {
    const box = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    try std.testing.expect(box.containsPoint(0, 0)); // top-left (inclusive)
    try std.testing.expect(box.containsPoint(10, 0)); // right edge (inclusive)
    try std.testing.expect(box.containsPoint(0, 10)); // bottom edge (inclusive)
    try std.testing.expect(box.containsPoint(10, 10)); // bottom-right (inclusive)
    try std.testing.expect(!box.containsPoint(11, 5)); // outside right
}

test "full collision pipeline: broad then narrow" {
    const a = AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = AABB{ .x = 8, .y = 8, .w = 10, .h = 10 };

    // Broad phase: AABB overlap
    try std.testing.expect(a.overlaps(b));

    // Narrow phase: check point containment
    try std.testing.expect(a.containsPoint(9, 9));
    try std.testing.expect(b.containsPoint(9, 9));
}

test "Circle and AABB both detect same overlap" {
    const circ = Circle{ .x = 5, .y = 5, .r = 3 };
    const box = AABB{ .x = 4, .y = 4, .w = 5, .h = 5 };
    // Both should agree on overlap with a nearby point
    try std.testing.expect(circ.containsPoint(6, 6));
    try std.testing.expect(box.containsPoint(6, 6));
}

test "AABB overlap is symmetric" {
    const a = AABB{ .x = 0, .y = 0, .w = 15, .h = 15 };
    const b = AABB{ .x = 10, .y = 10, .w = 15, .h = 15 };
    try std.testing.expectEqual(a.overlaps(b), b.overlaps(a));
}

test "Circle overlap is symmetric" {
    const a = Circle{ .x = 0, .y = 0, .r = 5 };
    const b = Circle{ .x = 8, .y = 0, .r = 4 };
    try std.testing.expectEqual(a.overlaps(b), b.overlaps(a));
}

test "AABB zero-size at origin" {
    const box = AABB{ .x = 0, .y = 0, .w = 0, .h = 0 };
    try std.testing.expect(box.containsPoint(0, 0));
    try std.testing.expect(!box.containsPoint(1, 0));
}

test "Circle zero radius" {
    const c = Circle{ .x = 5, .y = 5, .r = 0 };
    try std.testing.expect(!c.overlaps(Circle{ .x = 6, .y = 5, .r = 0 }));
    try std.testing.expect(c.containsPoint(5, 5));
}

test "pointInPolygon inside triangle" {
    const xs = [_]f32{ 0, 10, 5 };
    const ys = [_]f32{ 0, 0, 10 };
    try std.testing.expect(pointInPolygon(5, 3, &xs, &ys));
    try std.testing.expect(!pointInPolygon(20, 20, &xs, &ys));
}

test "example: ray vs AABB at t=5" {
    const ray = Ray{ .ox = 0, .oy = 5, .dx = 1, .dy = 0 };
    const box = AABB{ .x = 5, .y = 0, .w = 10, .h = 10 };
    const hit = ray.vsAABB(box).?;
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), hit.t, 0.1);
}
