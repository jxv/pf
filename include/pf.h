#ifndef PF_H
#define PF_H

#include <ml.h>

// Axis-aligned-bounded-box for quick collision detections.
typedef struct {
    v2f min;
    v2f max;
} pf_aabb;

// Circle is a supported shape.
typedef struct {
    v2f center;
    float radius;
} pf_circle;

// Because pf doesn't support rotations,
// the face's information can be stored and accessed quickly.
// Normal is the direction away from the face.
typedef struct {
    float angle;
    float sin;
    float cos;
    float len;
    v2f normal;
} pf_face;

// The polypair's face must be computed with another point.
// Do not assign values manually.
typedef struct {
    v2f point;
    pf_face face;
} pf_polypair;

// Polypair constructor
pf_polypair _pf_polypair(v2f point);

// Polygons are defined as collection of (preallocated) polypairs.
// Each face is computed by both its own pair's point and the following pair's point.
// Faces on a polygon must be ordered counter-clockwise.
// Because polygon points all must connect via faces,
// the last face must use the first pair's point as its other point.
typedef struct {
    pf_polypair *pairs;
    int count;
} pf_polygon;

// Polygon constructor
pf_polygon _pf_polygon(pf_polypair *pp, int count);

// Compute and store face-AB's information
// Face's angle is always non-negative.
void pf_compute_face(pf_face *f, const v2f *a, const v2f *b);

// Tagged union
typedef enum {
    PF_SHAPE_CIRCLE,
    PF_SHAPE_POLYGON,
} pf_shape_tag;

typedef struct {
    pf_shape_tag tag;
    union {
        pf_circle circle;
        pf_polygon polygon;
    };
} pf_shape;

// Collision tests
// Note: Overlapping borders should not register as a collision.
//       This is because of the difficulty of finding normals between exacts points on tagnents.
bool pf_test_aabb_vs_aabb(const pf_aabb *a, const pf_aabb *b);
bool pf_test_point_vs_aabb(const v2f *a, const pf_aabb *b);
bool pf_test_circle_vs_circle(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_circle *b);

// Collision match ups.
// Normal is the B into A penetration.
bool pf_shape_vs_shape(const v2f *a_pos, const pf_shape *a, const v2f *b_pos, const pf_shape *b, v2f *normal, float *penetration);
bool pf_circle_vs_circle(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_circle *b, v2f *normal, float *penetration);
bool pf_circle_vs_polygon(const v2f *a_pos, const pf_circle *a, const v2f *b_pos, const pf_polygon *b, v2f *normal, float *penetration);
bool pf_polygon_vs_polygon(const v2f *a_pos, const pf_polygon *a, const v2f *b_pos, const pf_polygon *b, v2f *normal, float *penetration);
bool pf_polygon_vs_circle(const v2f *a_pos, const pf_polygon *a, const v2f *b_pos, const pf_circle *b, v2f *normal, float *penetration);
bool pf_polygon_vs_point(const v2f *a_pos, const pf_polygon *a, const v2f *b_pos, v2f *normal, float *penetration);
bool pf_point_vs_polygon(const v2f *a_pos, const v2f *b_pos, const pf_polygon *b, v2f *normal, float *penetration);

// For the direction around a polygon/platform.
// With typical direction of gravity, clockwise = right, counter-clockwise = left.
typedef enum {
    PF_CIRCULAR_DIR_CLOCKWISE,
    PF_CIRCULAR_DIR_COUNTER_CLOCKWISE,
} pf_circular_dir;

// The referred point from a face.
typedef enum {
    PF_FACE_POINT_A,
    PF_FACE_POINT_B,
} pf_face_point;

// A face point's index from a platform perspective.
typedef struct {
    bool is;
    int polygon_index;
    int face_index;
    pf_face_point point;
} pf_platform_face_point;

// Act as a continuous platform bewteen faces.
// Describe the connection between points from different faces.
typedef struct {
    pf_platform_face_point a;
    pf_platform_face_point b;
} pf_platform_bind;

pf_platform_bind _pf_platform_bind_ab(
    int a_polygon_index, int a_face_index, pf_face_point a_point,
    int b_polygon_index, int b_face_index, pf_face_point b_point);

pf_platform_bind _pf_platform_bind_a(int a_polygon_index, int a_face_index, pf_face_point a_point);
pf_platform_bind _pf_platform_bind_b(int b_polygon_index, int b_face_index, pf_face_point b_point);


// A data extension of pf_polygon because polygons can bind.
// Count for binds is in the pf_polygon.
// The binds must be preallocated to the size of the count.
typedef struct {
    pf_polygon polygon;
    pf_platform_bind *binds;
} pf_platform_polygon;

// Platforms must support concave polygons, which can be imitated with many convex polygons.
// Those face points between different convex polygons must be bound via pf_platform_bind's.
typedef struct {
    pf_platform_polygon *polygons;
    int count;
} pf_platform;

#endif
