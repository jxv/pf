#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include "pf.h"

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


int main(void) {
    const struct CMUnitTest aabb_tests[] = {
        cmocka_unit_test(eq_pos_test_aabb_vs_aabb_test),

        cmocka_unit_test(inside_up_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(border_up_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(outside_up_pos_test_aabb_vs_aabb_test),

        cmocka_unit_test(inside_down_pos_test_aabb_vs_aabb_test),
        cmocka_unit_test(border_down_pos_test_aabb_vs_aabb_test),
    };
    return cmocka_run_group_tests(aabb_tests, NULL, NULL);
}
