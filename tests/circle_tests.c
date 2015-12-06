#include "tests.h"

void eq_test_circle_vs_circle_test(void **state) {

    const v2f a_pos = _v2f(0,0);

    const pf_circle a = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &a_pos, &a), true);
}

void up_test_circle_vs_circle_test(void **state) {
    const v2f a_pos = _v2f(0,0);
    const pf_circle a = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    const v2f inside_pos = _v2f(0, -2 + 0.1);
    const pf_circle inside = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    const v2f border_pos = _v2f(0, -2 + 0.0);
    const pf_circle border = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    const v2f outside_pos = _v2f(0, -2 - 0.1);
    const pf_circle outside = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &inside_pos, &inside), true);
    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &border_pos, &border), true);
    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &outside_pos, &outside), false);
}

void dr_test_circle_vs_circle_test(void **state) {
    const v2f a_pos = _v2f(0,0);

    const pf_circle a = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    const v2f border_pos = _v2f(sqrtf(2), sqrtf(2));
    const pf_circle border = {
        .center = _v2f(0,0),
        .radius = 1.0,
    };

    const v2f outside_pos = _v2f(sqrtf(2) + 0.1, sqrtf(2) + 0.1);
    const pf_circle outside = {
        .center = _v2f(0,0),
        .radius = 1.0,
    };

    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &border_pos, &border), true);
    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &outside_pos, &outside), false);
}

int run_circle_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(eq_test_circle_vs_circle_test),
        cmocka_unit_test(up_test_circle_vs_circle_test),
        cmocka_unit_test(dr_test_circle_vs_circle_test),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
