const std = @import("std");
const zc = @import("ziocollide");

pub fn main() !void {
    const a = zc.AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = zc.AABB{ .x = 5, .y = 5, .w = 10, .h = 10 };
    std.debug.print("AABB overlap: {}\n", .{a.overlaps(b)});

    const c = zc.Circle{ .x = 5, .y = 5, .r = 3 };
    std.debug.print("Circle contains (3,4): {}\n", .{c.containsPoint(3, 4)});

    std.debug.print("AABB vs Circle: {}\n", .{zc.aabbVsCircle(a, c)});

    const ray = zc.Ray{ .ox = -5, .oy = 5, .dx = 1, .dy = 0 };
    if (ray.vsAABB(a)) |hit| {
        std.debug.print("Ray hit AABB at t={d:.2}\n", .{hit.t});
    }
}
