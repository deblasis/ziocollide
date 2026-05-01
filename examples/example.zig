const std = @import("std");
const collide = @import("ziocollide");

pub fn main() !void {
    // AABB vs AABB
    const a = collide.AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = collide.AABB{ .x = 5, .y = 5, .w = 10, .h = 10 };
    std.debug.print("AABB overlaps: {}\n", .{a.overlaps(b)});
    std.debug.print("AABB containsPoint(7,7): {}\n", .{a.containsPoint(7, 7)});

    // AABB vs Circle
    const circ = collide.Circle{ .x = 5, .y = 5, .r = 3 };
    std.debug.print("AABB vs Circle: {}\n", .{collide.aabbVsCircle(a, circ)});
    std.debug.print("Circle containsPoint(3,4): {}\n", .{circ.containsPoint(3, 4)});

    // Point in polygon
    const xs = [_]f32{ 0, 10, 5 };
    const ys = [_]f32{ 0, 0, 10 };
    std.debug.print("pointInPolygon(5,3): {}\n", .{collide.pointInPolygon(5, 3, &xs, &ys)});

    // Ray casting
    const ray = collide.Ray{ .ox = 0, .oy = 5, .dx = 1, .dy = 0 };
    if (ray.vsAABB(a)) |hit| {
        std.debug.print("Ray hit AABB at t={d:.2}\n", .{hit.t});
    }

    // SAT overlap for convex polygons
    const p1x = [_]f32{ 0, 10, 10, 0 };
    const p1y = [_]f32{ 0, 0, 10, 10 };
    const p2x = [_]f32{ 5, 15, 15, 5 };
    const p2y = [_]f32{ 5, 5, 15, 15 };
    const depth = collide.satOverlap(&p1x, &p1y, &p2x, &p2y);
    std.debug.print("SAT overlap depth: {d:.2}\n", .{depth});
}
