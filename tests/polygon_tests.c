#include "tests.h"

void test_compute_face(void **state) {
    const v2f origin = _v2f(0, 0);
    const v2f left = _v2f(-1, 0);
    pf_face face;

    { 
        pf_compute_face(&face, &origin, &left);
        assert_true(eqf(face.angle, M_PI));
        assert_true(eqf(face.sin, 0));
        assert_true(eqf(face.cos, -1));
        assert_true(eqf(face.len, 1));
        assert_true(eqv2f(face.normal, _v2f(0,-1)));
    }
}

int run_polygon_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_compute_face),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
