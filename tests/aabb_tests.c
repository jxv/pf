#include "tests.h"

void eq_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &a), true);
}

void up_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb inside = {
        .min = _v2f(-1.0, -3.0 + 0.1),
        .max = _v2f( 1.0, -1.0 + 0.1),
    };

    const pf_aabb border = {
        .min = _v2f(-1.0, -3.0 + 0.0),
        .max = _v2f( 1.0, -1.0 + 0.0),
    };

    const pf_aabb outside = {
        .min = _v2f(-1.0, -3.0 - 0.1),
        .max = _v2f( 1.0, -1.0 - 0.1),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &inside), true);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &border), false);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &outside), false);
}

void down_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb inside = {
        .min = _v2f(-1.0, 1.0 - 0.1),
        .max = _v2f( 1.0, 3.0 - 0.1),
    };

    const pf_aabb border = {
        .min = _v2f(-1.0, 1.0 + 0.0),
        .max = _v2f( 1.0, 3.0 + 0.0),
    };

    const pf_aabb outside = {
        .min = _v2f(-1.0, 1.0 + 0.1),
        .max = _v2f( 1.0, 3.0 + 0.1),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &inside), true);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &border), false);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &outside), false);
}

void left_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb inside = {
        .min = _v2f(-3.0 + 0.1, -1.0),
        .max = _v2f(-1.0 + 0.1,  1.0),
    };
    
    const pf_aabb border = {
        .min = _v2f(-3.0 + 0.0, -1.0),
        .max = _v2f(-1.0 + 0.0,  1.0),
    };
    
    const pf_aabb outside = {
        .min = _v2f(-3.0 - 0.1, -1.0),
        .max = _v2f(-1.0 - 0.1,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &inside), true);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &border), false);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &outside), false);
}

void right_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb inside = {
        .min = _v2f(1.0 - 0.1, -1.0),
        .max = _v2f(3.0 - 0.1,  1.0),
    };
    
    const pf_aabb border = {
        .min = _v2f(1.0 - 0.0, -1.0),
        .max = _v2f(3.0 - 0.0,  1.0),
    };
    
    const pf_aabb outside = {
        .min = _v2f(1.0 + 0.1, -1.0),
        .max = _v2f(3.0 + 0.1,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &inside), true);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &border), false);
    assert_int_equal(pf_test_aabb_vs_aabb(&a, &outside), false);
}

void center_point_vs_aabb_test(void **state) {
     const v2f center = _v2f(0, 0);
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };    

    assert_int_equal(pf_test_point_vs_aabb(&center, &b), true);
}

void up_point_vs_aabb_test(void **state) {
     const v2f outside = _v2f(0, -1.0 - 0.1);
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };    

    assert_int_equal(pf_test_point_vs_aabb(&outside, &b), false);
}

void down_point_vs_aabb_test(void **state) {
     const v2f outside = _v2f(0, 1.0 + 0.1);
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };    

    assert_int_equal(pf_test_point_vs_aabb(&outside, &b), false);
}

void left_point_vs_aabb_test(void **state) {
     const v2f outside = _v2f(-1.0 - 0.1, 0);
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };    

    assert_int_equal(pf_test_point_vs_aabb(&outside, &b), false);
}

int run_aabb_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(eq_test_aabb_vs_aabb_test),
        cmocka_unit_test(up_test_aabb_vs_aabb_test),
        cmocka_unit_test(down_test_aabb_vs_aabb_test),
        cmocka_unit_test(left_test_aabb_vs_aabb_test),
        cmocka_unit_test(right_test_aabb_vs_aabb_test),

        cmocka_unit_test(center_point_vs_aabb_test),
        cmocka_unit_test(up_point_vs_aabb_test),
        cmocka_unit_test(down_point_vs_aabb_test),
        cmocka_unit_test(left_point_vs_aabb_test),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
