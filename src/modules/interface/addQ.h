#pragma once

#include "ekfutil.h"
#include "mexutil.h"

#define EKF_DISTURBANCE (6)

static void set_G_block33(float G[EKF_N][EKF_DISTURBANCE], int row, int col, struct mat33 block)
{
	float *blockptr = &G[row][col];
	set_block33(blockptr, EKF_DISTURBANCE, block);
}

void addQ(double dt, struct quat q, struct vec ew, struct vec ea, float Q[EKF_N][EKF_N])
{
	// optimize diagonal mmult or maybe A Diag A' ?
	static float Qdiag[EKF_DISTURBANCE][EKF_DISTURBANCE];
	ZEROARR(Qdiag);
	Qdiag[0][0] = Qdiag[1][1] = Qdiag[2][2] = GYRO_VAR_XYZ;
	Qdiag[3][3] = Qdiag[4][4] = Qdiag[5][5] = ACC_VAR_XYZ;

	static float G[EKF_N][EKF_DISTURBANCE];
	ZEROARR(G);
	set_G_block33(G, 6, 0, eyescl(-1));
	set_G_block33(G, 3, 3, mneg(quat2rotmat(qinv(q))));

	static float QGt[EKF_DISTURBANCE][EKF_N];
	ZEROARR(QGt);
	SGEMM2D('n', 't', EKF_DISTURBANCE, EKF_N, EKF_DISTURBANCE, 1.0, Qdiag, G, 0.0, QGt);

	SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_DISTURBANCE, dt, G, QGt, 1.0, Q);

	// debug only
	//float Qd[EKF_N][EKF_N];
	//ZEROARR(Qd);
	//SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_DISTURBANCE, 1.0, G, QGt, 0.0, Qd);
	//checksym("Q", AS_1D(Qd), EKF_N);
}
