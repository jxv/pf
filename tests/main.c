#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include "pf.h"

void test_aabb_vs_aabb_success(void **state) {
    (void)state;

    // Same aabb
    {
        pf_aabb a = {
            .min = _v2f(-1.0, -1.0),
            .max = _v2f( 1.0,  1.0),
        };

        assert_int_equal(pf_test_aabb_vs_aabb(&a, &a), true);
    }
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_aabb_vs_aabb_success),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
