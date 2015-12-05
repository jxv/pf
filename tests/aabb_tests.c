#include "tests.h"

void eq_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &a), true);
}

void inside_up_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -2.0),
        .max = _v2f( 1.0,  0.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void border_up_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -3.0),
        .max = _v2f( 1.0, -1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void outside_up_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -4.0),
        .max = _v2f( 1.0, -2.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), false);
}

void inside_down_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-1.0, 0.0),
        .max = _v2f( 1.0, 2.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void border_down_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-1.0, 1.0),
        .max = _v2f( 1.0, 3.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void outside_down_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-1.0, 2.0),
        .max = _v2f( 1.0, 4.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), false);
}

void inside_left_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-2.0, -1.0),
        .max = _v2f( 0.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void border_left_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-3.0, -1.0),
        .max = _v2f(-1.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void outside_left_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f(-4.0, -1.0),
        .max = _v2f(-2.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), false);
}

void inside_right_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f( 0.0, -1.0),
        .max = _v2f( 2.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void border_right_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f( 1.0, -1.0),
        .max = _v2f( 3.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), true);
}

void outside_right_pos_test_aabb_vs_aabb_test(void **state) {
    const pf_aabb a = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };
    
    const pf_aabb b = {
        .min = _v2f( 2.0, -1.0),
        .max = _v2f( 4.0,  1.0),
    };

    assert_int_equal(pf_test_aabb_vs_aabb(&a, &b), false);
}

void inside_point_vs_aabb_test(void **state) {
     const v2f a = _v2f(0, 0);
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };    

    assert_int_equal(pf_test_point_vs_aabb(&a, &b), true);
}

void outside_up_point_vs_aabb_test(void **state) {
     const v2f a = _v2f(0, -2);
    
    const pf_aabb b = {
        .min = _v2f(-1.0, -1.0),
        .max = _v2f( 1.0,  1.0),
    };    

    assert_int_equal(pf_test_point_vs_aabb(&a, &b), false);
}

int run_aabb_tests() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(eq_pos_test_aabb_vs_aabb_test),

        cmocka_unit_test(inside_up_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(border_up_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(outside_up_pos_test_aabb_vs_aabb_test),

        cmocka_unit_test(inside_down_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(border_down_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(outside_down_pos_test_aabb_vs_aabb_test),

        cmocka_unit_test(inside_left_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(border_left_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(outside_left_pos_test_aabb_vs_aabb_test),
        
        cmocka_unit_test(inside_right_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(border_right_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(outside_right_pos_test_aabb_vs_aabb_test),
        
        cmocka_unit_test(inside_point_vs_aabb_test),
        cmocka_unit_test(outside_up_point_vs_aabb_test),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
