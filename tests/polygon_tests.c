#include <stdio.h>
#include "tests.h"

void test_left_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f left = _v2f(-1, 0);
    pf_face face;
    pf_compute_face(&face, &origin, &left);
    assert_true(eqf(face.angle, M_PI));
    assert_true(eqf(face.sin, 0));
    assert_true(eqf(face.cos, -1));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(0,-1)));
}

void test_right_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f right = _v2f(1, 0);
    pf_face face;
    pf_compute_face(&face, &origin, &right);
    assert_true(eqf(face.angle, 0));
    assert_true(eqf(face.sin, 0));
    assert_true(eqf(face.cos, 1));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(0, 1)));
}

void test_up_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f up = _v2f(0, -1);
    pf_face face;
    pf_compute_face(&face, &origin, &up);
    assert_true(eqf(face.angle, M_PI_2 * 3));
    assert_true(eqf(face.sin, -1));
    assert_true(eqf(face.cos, 0));
    assert_true(eqf(face.len, 1));
    assert_true(eqv2f(face.normal, _v2f(1, 0)));
}

void test_dr_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f dr = _v2f(sqrtf(2) / 2, sqrtf(2) / 2);
    pf_face face;
    pf_compute_face(&face, &origin, &dr);
    const float angle = M_PI_4;
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
    const float angle = M_PI_4 * 5;
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

int run_polygon_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_left_compute_face),
        cmocka_unit_test(test_right_compute_face),
        cmocka_unit_test(test_up_compute_face),
        cmocka_unit_test(test_dr_compute_face),
        cmocka_unit_test(test_ul_compute_face),
        cmocka_unit_test(test_pf_polypair),
        cmocka_unit_test(test_pf_platform_bind_ab),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
