#include "tests.h"

int (*runners[])() = {
    run_aabb_tests,
    run_circle_tests,
};

int main() {
    for (int i = 0; runners[i] != NULL; i++) {
        int res = runners[i]();
        if (res) {
            return res;
        }
    }
    return 0;
}
