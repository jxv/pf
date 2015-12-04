#include "tests.h"

void eq_pos_test_circle_vs_circle_test(void **state) {

    const v2f a_pos = _v2f(0,0);

    const pf_circle a = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &a_pos, &a), true);
}

void border_up_pos_test_circle_vs_circle_test(void **state) {
    const v2f a_pos = _v2f(0,0);

    const pf_circle a = {
        .center = _v2f(0, 0),
        .radius = 1.0,
    };

    const v2f b_pos = _v2f(0,0);

    const pf_circle b = {
        .center = _v2f(0, 1),
        .radius = 1.0,
    };

    assert_int_equal(pf_test_circle_vs_circle(&a_pos, &a, &b_pos, &b), true);
}

int run_circle_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(eq_pos_test_circle_vs_circle_test),
        cmocka_unit_test(border_up_pos_test_circle_vs_circle_test),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
