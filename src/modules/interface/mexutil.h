#pragma once

#include <math.h>

#ifdef MATLAB_MEX_FILE
#include <mex.h>
#define MEXUTIL_PRINTF mexPrintf
#else
//#include <stdio.h>
#define MEXUTIL_PRINTF
#endif

#define mxPrintVec(VAR) MEXUTIL_PRINTF("%s: (%f, %f, %f)\n", #VAR, (VAR).x, (VAR).y, (VAR).z);

static void checknan(char const *name, float const *f, int n)
{
	for (int i = 0; i < n; ++i) {
		if (!isfinite(f[i]) && (f[i] == f[i])) {
			MEXUTIL_PRINTF("%s has infinity\n", name);
			return;
		}
		if (f[i] != f[i]) {
			MEXUTIL_PRINTF("%s has NaN\n", name);
			return;
		}
	}
}
