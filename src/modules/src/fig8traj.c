/**
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *
 * Copyright (C) 2016 Wolfgang Hoenig and James Preiss,
 * University of Southern California
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * pptraj.c: implementation of xyz-yaw piecewise polynomial trajectories
 *           see Mellinger and Kumar, "Minimum Snap...", ICRA 2011
 */

#include "pptraj.h"

struct piecewise_traj pp_figure8 = 
{
.pieces = {
	{
	.p = {{-0.000000,0.000000,-0.000000,0.480798,0.022675,-0.128325,-0.063306,0.043016,},
	{0.000000,-0.000000,0.000000,-0.998070,0.319252,0.383615,-0.079148,-0.043921,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 1.050000
	},

	{
	.p = {{0.396058,0.834075,0.177659,-0.474431,0.044917,0.006196,0.023509,-0.012529,},
	{-0.445604,-0.509485,0.788518,0.871157,-0.755213,-0.077059,0.058616,0.057071,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.710000
	},

	{
	.p = {{0.922409,0.455261,-0.632709,-0.258786,0.087728,0.107725,0.045679,-0.057769,},
	{-0.291165,0.863185,0.523727,-0.734922,0.129373,0.009896,0.008797,-0.023621,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.620000
	},

	{
	.p = {{0.923174,-0.462370,-0.664872,0.291840,0.203181,-0.052775,-0.092636,0.030602,},
	{0.289869,0.791184,-0.547199,-0.456261,0.013746,0.138316,0.098124,-0.077992,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.700000
	},

	{
	.p = {{0.405364,-0.816991,0.138708,0.223831,-0.295030,-0.045158,0.026762,0.062636,},
	{0.450742,-0.425968,-0.912384,0.277703,0.282769,0.017778,-0.047575,-0.033170,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.560000
	},

	{
	.p = {{0.001062,-0.658146,-0.007770,-0.269057,0.089414,0.098149,0.032476,-0.047275,},
	{0.001593,-1.002081,0.008890,0.685582,-0.095129,-0.061464,-0.028222,0.037227,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.560000
	},

	{
	.p = {{-0.402804,-0.808380,-0.125946,0.190369,0.226421,-0.022765,-0.069116,0.012615,},
	{-0.449354,-0.445467,0.875153,0.308753,-0.171169,-0.146177,-0.058826,0.079921,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.700000
	},

	{
	.p = {{-0.921641,-0.480850,0.657024,0.344668,-0.209824,-0.096006,-0.021079,0.060385,},
	{-0.292459,0.829781,0.589126,-0.618718,-0.155707,0.074789,0.087006,-0.034754,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.620000
	},

	{
	.p = {{-0.923935,0.472794,0.654811,-0.332923,-0.125283,0.028854,0.060527,-0.026578,},
	{0.288570,0.787618,-0.616585,-0.482397,0.287922,0.070102,0.000375,-0.034362,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 0.710000
	},

	{
	.p = {{-0.398611,0.798140,-0.199957,-0.346390,0.101890,0.024508,0.025445,-0.005129,},
	{0.447039,-0.346597,-0.651106,0.385746,0.109165,0.156675,0.044322,-0.144883,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,}},
	.duration = 1.053185
	}
},
.n_pieces = 10
};
