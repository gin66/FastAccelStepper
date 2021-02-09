#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "RampCalculator.h"

// Not a real test case

int main() {
    uint32_t res;

	// Calculation is pre_calc/sqrt(steps)
	//
    uint32_t pre_calc = 10000000;	
    uint32_t steps = 100000;	
	float correct;
	float err;
	for (int mode = 0;mode <= 1;mode++) {
		upm_float upm_pre_calc = upm_from(pre_calc);
		res = calculate_ticks_v8(steps, upm_pre_calc, mode == 1);
		correct = upm_to_u32(upm_pre_calc) * 1.0 / sqrt(steps);
		err = res - correct;
		printf("%d %f  %f\n",res,correct,err);
	}

//	assert(false);
	return 0;
}
