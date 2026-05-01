# ziocollide

> 2D collision detection for Zig: AABB, circle, ray, point-in-polygon, SAT. Zero allocation.

Part of the [zio-zig](https://github.com/deblasis/zio-zig) ecosystem.

## Quick start

```zig
const collide = @import("ziocollide");

// AABB vs AABB
const a = collide.AABB{ .x = 0, .y = 0, .w = 10, .h = 10 };
const b = collide.AABB{ .x = 5, .y = 5, .w = 10, .h = 10 };
if (a.overlaps(b)) {
    // resolve collision...
}

// AABB vs Circle
const circ = collide.Circle{ .x = 5, .y = 5, .r = 3 };
if (collide.aabbVsCircle(a, circ)) { /* hit */ }

// Point in polygon
const xs = [_]f32{ 0, 10, 5 };
const ys = [_]f32{ 0, 0, 10 };
if (collide.pointInPolygon(5, 3, &xs, &ys)) { /* inside */ }

// Ray casting
const ray = collide.Ray{ .ox = 0, .oy = 0, .dx = 1, .dy = 0 };
if (ray.vsCircle(circ)) |hit| {
    std.debug.print("hit at t={d}\n", .{hit.t});
}

// SAT overlap for convex polygons
const depth = collide.satOverlap(&poly1_x, &poly1_y, &poly2_x, &poly2_y);
```

```bash
zig build test          # Run 40 tests
zig build run-example   # Run example
```

## Example output

```
$ zig build run-example
AABB overlaps: true
AABB containsPoint(7,7): true
AABB vs Circle: true
Circle containsPoint(3,4): true
pointInPolygon(5,3): true
Ray hit AABB at t=0.00
SAT overlap depth: 5.00
```

## API

### Types

| Type | Fields | Description |
|------|--------|-------------|
| `AABB` | `x, y, w, h` | Axis-aligned bounding box |
| `Circle` | `x, y, r` | Circle |
| `Ray` | `ox, oy, dx, dy` | Ray with origin and direction |
| `RayHit` | `t, nx, ny` | Ray hit result (distance + normal) |

### AABB methods
- `overlaps(b)` — overlap test
- `containsPoint(px, py)` — point containment
- `minX/Y()`, `maxX/Y()` — boundary accessors

### Circle methods
- `overlaps(b)` — circle-circle overlap
- `containsPoint(px, py)` — point in circle

### Ray methods
- `vsCircle(c)` — ray-circle intersection, returns `?RayHit`
- `vsAABB(box)` — ray-AABB intersection, returns `?RayHit`

### Free functions
- `aabbVsCircle(box, circ)` — AABB-circle overlap
- `pointInPolygon(px, py, poly_x, poly_y)` — point-in-polygon test
- `satOverlap(ax, ay, bx, by)` — separating axis theorem overlap depth

## License

MIT. Copyright (c) 2026 Alessandro De Blasis.
