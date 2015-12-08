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
    assert_true(eqf(face.angle, -M_PI_2));
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

int run_polygon_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_left_compute_face),
        cmocka_unit_test(test_right_compute_face),
        cmocka_unit_test(test_up_compute_face),
        cmocka_unit_test(test_dr_compute_face),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
