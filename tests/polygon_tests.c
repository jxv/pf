#include <stdio.h>
#include "tests.h"

void test_left_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f left = _v2f(-1, 0);
    pf_face face;
    pf_compute_face(&face, &origin, &left);
    assert_true(eqf(face.angle, 0));
    assert_true(eqf(face.sin, 0));
    assert_true(eqf(face.cos, 1));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(0,-1)));
}

void test_right_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f right = _v2f(1, 0);
    pf_face face;
    pf_compute_face(&face, &origin, &right);
    assert_true(eqf(face.angle, M_PI));
    assert_true(eqf(face.sin, 0));
    assert_true(eqf(face.cos,-1));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(0, 1)));
}

void test_up_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f up = _v2f(0, -1);
    pf_face face;
    pf_compute_face(&face, &origin, &up);
    assert_true(eqf(face.angle, M_PI_2));
    assert_true(eqf(face.sin, 1));
    assert_true(eqf(face.cos, 0));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(1, 0)));
}

void test_dr_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f dr = _v2f(sqrtf(2) / 2, sqrtf(2) / 2);
    pf_face face;
    pf_compute_face(&face, &origin, &dr);
    const float angle = M_PI_4 * 5;
    assert_true(eqf(face.angle, angle));
    assert_true(eqf(face.sin, sinf(angle)));
    assert_true(eqf(face.cos, cosf(angle)));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(-sqrtf(2) / 2, sqrtf(2) / 2)));
}

void test_ul_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f ul = _v2f(-sqrtf(2) / 2, -sqrtf(2) / 2);
    pf_face face;
    pf_compute_face(&face, &origin, &ul);
    const float angle = M_PI_4;
    assert_true(eqf(face.angle, angle));
    assert_true(eqf(face.sin, sinf(angle)));
    assert_true(eqf(face.cos, cosf(angle)));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(sqrtf(2) / 2, -sqrtf(2) / 2)));
}

void test_pf_polypair(void **state) {
    const v2f point = _v2f(12,34);
    const pf_polypair pair = _pf_polypair(point);
    assert_true(eqv2f(point, pair.point));
    assert_true(pair.face.angle == 0);
    assert_true(pair.face.sin == 0);
    assert_true(pair.face.cos == 0);
    assert_true(pair.face.len == 0);
    assert_true(pair.face.normal.x == 0);
    assert_true(pair.face.normal.y == 0);
}

void test_pf_polygon(void **state) {
#define COUNT 4
    pf_polypair pairs[COUNT] = {
        _pf_polypair(_v2f(-1, -1)),
        _pf_polypair(_v2f(-1,  1)),
        _pf_polypair(_v2f( 1,  1)),
        _pf_polypair(_v2f( 1, -1)),
    };
    pf_polygon polygon = _pf_polygon(pairs, COUNT);

    assert_int_equal(polygon.count, COUNT);
#undef COUNT
    
    assert_true(eqf(polygon.pairs[0].face.len, 2));
    assert_true(eqf(polygon.pairs[1].face.len, 2));
    assert_true(eqf(polygon.pairs[2].face.len, 2));
    assert_true(eqf(polygon.pairs[3].face.len, 2));
    
    assert_true(eqf(polygon.pairs[0].face.angle, M_PI_2 * 3));
    assert_true(eqf(polygon.pairs[1].face.angle, M_PI));
    assert_true(eqf(polygon.pairs[2].face.angle, M_PI_2));
    assert_true(eqf(polygon.pairs[3].face.angle, 0));

    assert_true(eqv2f(polygon.pairs[0].face.normal, _v2f(-1,  0)));
    assert_true(eqv2f(polygon.pairs[1].face.normal, _v2f( 0,  1)));
    assert_true(eqv2f(polygon.pairs[2].face.normal, _v2f( 1,  0)));
    assert_true(eqv2f(polygon.pairs[3].face.normal, _v2f( 0, -1)));
}

void test_pf_platform_bind_ab(void **state) {
    pf_platform_bind ab = _pf_platform_bind_ab(1, 2, PF_FACE_POINT_B, 3, 4, PF_FACE_POINT_A);
    assert_true(ab.a.is);
    assert_int_equal(ab.a.polygon_index, 1);
    assert_int_equal(ab.a.face_index, 2);
    assert_int_equal(ab.a.point, PF_FACE_POINT_B);
    assert_true(ab.b.is);
    assert_int_equal(ab.b.polygon_index, 3);
    assert_int_equal(ab.b.face_index, 4);
    assert_int_equal(ab.b.point, PF_FACE_POINT_A);
}

void test_pf_platform_bind_a(void **state) {
    pf_platform_bind a = _pf_platform_bind_a(1, 2, PF_FACE_POINT_B);
    assert_true(a.a.is);
    assert_int_equal(a.a.polygon_index, 1);
    assert_int_equal(a.a.face_index, 2);
    assert_int_equal(a.a.point, PF_FACE_POINT_B);
    assert_false(a.b.is);
    assert_int_equal(a.b.polygon_index, 0);
    assert_int_equal(a.b.face_index, 0);
    assert_int_equal(a.b.point, 0);
}

void test_pf_platform_bind_b(void **state) {
    pf_platform_bind b = _pf_platform_bind_b(3, 4, PF_FACE_POINT_A);
    assert_false(b.a.is);
    assert_int_equal(b.a.polygon_index, 0);
    assert_int_equal(b.a.face_index, 0);
    assert_int_equal(b.a.point, 0);
    assert_true(b.b.is);
    assert_int_equal(b.b.polygon_index, 3);
    assert_int_equal(b.b.face_index, 4);
    assert_int_equal(b.b.point, PF_FACE_POINT_A);
}

void test_traverse_box(void **state) {
#define COUNT 4
    pf_polypair pairs[COUNT] = {
        _pf_polypair(_v2f(-1, -1)),
        _pf_polypair(_v2f(-1,  1)),
        _pf_polypair(_v2f( 1,  1)),
        _pf_polypair(_v2f( 1, -1)),
    };
    pf_polygon polygon = _pf_polygon(pairs, COUNT);

    pf_platform_bind binds[COUNT] = {
        _pf_platform_bind_ab(0, 1, PF_FACE_POINT_B, 0, 3, PF_FACE_POINT_A),
        _pf_platform_bind_ab(0, 2, PF_FACE_POINT_B, 0, 0, PF_FACE_POINT_A),
        _pf_platform_bind_ab(0, 3, PF_FACE_POINT_B, 0, 1, PF_FACE_POINT_A),
        _pf_platform_bind_ab(0, 0, PF_FACE_POINT_B, 0, 2, PF_FACE_POINT_A),
    };

#undef COUNT

    pf_platform_polygon plat_poly = { polygon, binds };
    {
        v2f pos = polygon.pairs[0].point;
        (void) pos;
        pf_platform_face plat_face = { 0, 0 };
        v2f move = _v2f(2, 0);

        const pf_polypair *pp = &plat_poly.polygon.pairs[plat_face.face_index];
        v2f moved = pf_rotate_for_traverse(move, pp->face.sin, pp->face.cos);
        assert_true(eqv2f(moved, _v2f(0, -2)));
    }
}

int run_polygon_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_left_compute_face),
        cmocka_unit_test(test_right_compute_face),
        cmocka_unit_test(test_up_compute_face),
        cmocka_unit_test(test_dr_compute_face),
        cmocka_unit_test(test_ul_compute_face),
        cmocka_unit_test(test_pf_polypair),
        cmocka_unit_test(test_pf_polygon),
        cmocka_unit_test(test_pf_platform_bind_ab),
        cmocka_unit_test(test_pf_platform_bind_a),
        cmocka_unit_test(test_pf_platform_bind_b),
        cmocka_unit_test(test_traverse_box),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
