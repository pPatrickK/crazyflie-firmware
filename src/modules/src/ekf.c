#include <math.h>
#include <stdbool.h> // bool

#include "cholsl.h"
#include "ekf.h"
#include "mexutil.h"


// measured constants
#define VICON_VAR_XY 1.5e-9
#define VICON_VAR_Z  1.0e-8
#define VICON_VAR_Q  4.5e-6
#define GYRO_VAR_XYZ 0.2e-6
#define ACC_VAR_XY   1.5e-5
#define ACC_VAR_Z    3.9e-5
// the accelerometer variance in z was quite a bit higher
// but to keep the code simple for now we just average them
#define ACC_VAR_XYZ  2.4e-3

// ------ utility functions for manipulating blocks of the EKF matrices ------

void set_K_block33(float m[EKF_N][EKF_N], int row, int col, struct mat33 const *block)
{
	float *blockptr = &m[row][col];
	set_block33(blockptr, EKF_N, block);
}

void mult_K_block33(float m[EKF_N][EKF_N], int row, int col, struct mat33 const *a, struct mat33 const *b)
{
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float accum = 0;
			for (int k = 0; k < 3; ++k) {
				accum += a->m[i][k] * b->m[k][j];
			}
			m[row + i][col + j] = accum;
		}
	}
}

void set_H_block33(float h[EKF_M][EKF_N], int row, int col, struct mat33 const *block)
{
	float *blockptr = &h[row][col];
	set_block33(blockptr, EKF_N, block);
}

static void set_G_block33(float G[EKF_N][EKF_DISTURBANCE], int row, int col, struct mat33 const *block)
{
	float *blockptr = &G[row][col];
	set_block33(blockptr, EKF_DISTURBANCE, block);
}

// -------------------------- EKF implementation -----------------------------

// initialize the EKF struct
void ekf_init(struct ekf *ekf, float const pos[3], float const vel[3], float const quat[4])
{
	ekf->pos = vloadf(pos);
	ekf->vel = vloadf(vel);
	ekf->quat = qloadf(quat);
	//memcpy(ekf->P, ekf_cov_init, sizeof(ekf_cov_init));
	eyeN(AS_1D(ekf->P), EKF_N);
}

void dynamic_matrix(struct quat const q, struct vec const omega, struct vec const acc, float const dt, float F[EKF_N][EKF_N])
{
	float const dt_p2_2 = dt * dt * 0.5;
	float const dt_p3_6 = dt_p2_2 * dt / 3.0;
	//float const dt_p4_24 = dt_p3_6 * dt * 0.25;
	//float const dt_p5_120 = dt_p4_24 * dt * 0.2;

	struct mat33 C_eq = quat2rotmat(q);
	struct mat33 w_sk = crossmat(omega);
	struct mat33 a_sk = crossmat(acc);
	// TEMP DEBUG
	//struct vec acc_nograv = vsub(acc, qvrot(q, mkvec(0,0,GRAV)));
	//struct mat33 const a_sk = crossmat(acc_nograv);

	// TODO S.Weiss doesn't subtract gravity from the accelerations here, is that right?
	struct mat33 Ca3;
	mmultp(&C_eq, &a_sk, &Ca3);

	struct mat33 A = aXplusbI(dt_p3_6, &w_sk, -dt_p2_2); // position by quaternion
	struct mat33 E = aXplusbI(-dt, &w_sk, 1.0f); // quat by quat
	struct mat33 FF = aXplusbI(dt_p2_2, &w_sk, -dt); // quat by gyro bias

	eyeN(AS_1D(F), EKF_N);

	//set_K_block33(F, 0, 3, eyescl(dt));
	F[0][3] = F[1][4] = F[2][5] = dt;

	mult_K_block33(F, 0, 6, &Ca3, &A);

	mult_K_block33(F, 3, 6, &Ca3, &FF);

	set_K_block33(F, 6, 6, &E);
}

/* currently unused
static void symmetricize(float a[EKF_N][EKF_N])
{
	for (int i = 0; i < EKF_N; ++i) {
		for (int j = 0; j < EKF_N; ++j) {
			float mean = (a[i][j] + a[j][i]) / 2;
			a[i][j] = mean;
			a[j][i] = mean;
		}
	}
}
*/

void addQ(double dt, struct quat q, struct vec ew, struct vec ea, float Q[EKF_N][EKF_N])
{
	// optimize diagonal mmult or maybe A Diag A' ?
	static float Qdiag[EKF_DISTURBANCE][EKF_DISTURBANCE];
	ZEROARR(Qdiag);
	Qdiag[0][0] = Qdiag[1][1] = Qdiag[2][2] = GYRO_VAR_XYZ;
	Qdiag[3][3] = Qdiag[4][4] = Qdiag[5][5] = ACC_VAR_XYZ;

	static float G[EKF_N][EKF_DISTURBANCE];
	ZEROARR(G);
	struct mat33 quat_by_gyro = eyescl(-1);
	struct mat33 vel_by_acc = mneg(quat2rotmat(qinv(q)));
	set_G_block33(G, 6, 0, &quat_by_gyro);
	set_G_block33(G, 3, 3, &vel_by_acc);

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

void ekf_imu(struct ekf const *ekf_prev, struct ekf *ekf, float const acc[3], float const gyro[3], float dt)
{
	//------------------------- integrate dynamics --------------------------//
	// propagate constant states
	//ekf->bias_acc = ekf_prev->bias_acc;
	//ekf->bias_gyro = ekf_prev->bias_gyro;

	// TODO TEMP avoid dumb bugs
	*ekf = *ekf_prev;


	// TODO follow S.Weiss and use means with the previous IMU measurements?
	// or stick with the ultra-simple propagation here?
	// (I think the means is not that helpful)

	// propagate rotation
	// TODO only normalize every N steps to save computation?
	//struct vec const omega = vsub(vloadf(gyro), ekf->bias_gyro);
	struct vec const omega = vloadf(gyro);
	ekf->quat = qnormalized(quat_gyro_update(ekf_prev->quat, omega, dt));

	// compute true acceleration
	//struct vec const acc_imu = vsub(float2vec(acc), ekf->bias_acc);
	struct vec const acc_imu = vloadf(acc);
    struct vec acc_world = qvrot(ekf->quat, acc_imu);
    acc_world.z -= GRAV;

	// propagate position + velocity
	ekf->vel = vadd(ekf_prev->vel, vscl(dt, acc_world));
	ekf->pos = vadd(ekf_prev->pos, vscl(dt, ekf->vel));

	//-------------------------- update covariance --------------------------//
	// TODO should use old quat??
	static float F[EKF_N][EKF_N];
	dynamic_matrix(ekf->quat, omega, acc_imu, dt, F);

	// Pnew = F P Ft + Q
	static float PFt[EKF_N][EKF_N];
	ZEROARR(PFt);
	SGEMM2D('n', 't', EKF_N, EKF_N, EKF_N, 1.0, ekf_prev->P, F, 0.0, PFt);
	SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_N, 1.0, F, PFt, 0.0, ekf->P);
	addQ(dt, ekf->quat, omega, acc_imu, ekf->P);
	//symmetricize(ekf->P);
	checknan("P", AS_1D(ekf->P), EKF_N * EKF_N);
}

void ekf_vicon(struct ekf const *old, struct ekf *new, float const pos_vicon[3], float const quat_vicon[4])//, double *debug)
{
	*new = *old;

	struct vec const p_vicon = vloadf(pos_vicon);
	struct quat const q_vicon = qloadf(quat_vicon);

	struct quat const q_residual = qqmul(qinv(old->quat), q_vicon);
	struct vec const err_quat = vscl(2.0f / q_residual.w, q_residual.v);
	struct vec const err_pos = vsub(p_vicon, old->pos);

	float residual[EKF_M];
	vstoref(err_pos, residual);
	vstoref(err_quat, residual + 3);

	// TODO this matrix is just identity blocks
	// we should be able to hand-code the multiplication to be much more efficient
	static float H[EKF_M][EKF_N];
	ZEROARR(H);
	struct mat33 meye = eye();
	set_H_block33(H, 0, 0, &meye);
	set_H_block33(H, 3, 6, &meye);

	// S = H P H' + R  :  innovation

	static float PHt[EKF_N][EKF_M];
	ZEROARR(PHt);
	SGEMM2D('n', 't', EKF_N, EKF_M, EKF_N, 1.0, old->P, H, 0.0, PHt);

	static float S[EKF_M][EKF_M];
	ZEROARR(S);
	SGEMM2D('n', 'n', EKF_M, EKF_M, EKF_N, 1.0, H, PHt, 0.0, S);
	checknan("S", AS_1D(S), EKF_M * EKF_M);

	// diag only, no cov
	float const Rdiag[EKF_M] =
		{ VICON_VAR_XY, VICON_VAR_XY, VICON_VAR_XY, VICON_VAR_Q, VICON_VAR_Q, VICON_VAR_Q };
	static float R[EKF_M][EKF_M];
	ZEROARR(R);
	for (int i = 0; i < EKF_M; ++i) {
		S[i][i] += Rdiag[i];
		R[i][i] = Rdiag[i];
	}

	// K = P H' S^-1   :  gain

	static float Sinv[EKF_M][EKF_M];
	static float scratch[EKF_M];
	cholsl(AS_1D(S), AS_1D(Sinv), scratch, EKF_M);
	checknan("S^-1", AS_1D(Sinv), EKF_M * EKF_M);

	static float HtSinv[EKF_N][EKF_M];
	ZEROARR(HtSinv);
	SGEMM2D('t', 'n', EKF_N, EKF_M, EKF_M, 1.0, H, Sinv, 0.0, HtSinv);

	static float K[EKF_N][EKF_M];
	ZEROARR(K);
	SGEMM2D('n', 'n', EKF_N, EKF_M, EKF_N, 1.0, old->P, HtSinv, 0.0, K);
	checknan("K", AS_1D(K), EKF_N * EKF_M);


	// K residual : correction

	static float correction[EKF_N];
	ZEROARR(correction);
	sgemm('n', 'n', EKF_N, 1, EKF_M, 1.0, AS_1D(K), residual, 0.0, correction);

	new->pos = vadd(old->pos, vloadf(correction + 0));
	new->vel = vadd(old->vel, vloadf(correction + 3));
	struct quat error_quat = qrpy_small(vloadf(correction + 6));
	new->quat = qnormalized(qqmul(old->quat, error_quat));
	// TODO biases, if we use dem


	// Pnew = (I - KH) P (I - KH)^T + KRK^T  :  covariance update

	static float RKt[EKF_M][EKF_N];
	ZEROARR(RKt);
	SGEMM2D('n', 't', EKF_M, EKF_N, EKF_M, 1.0, R, K, 0.0, RKt);

	// KRKt - store in P so we can add to it in-place with SGEMM later
	SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_M, 1.0, K, RKt, 0.0, new->P);
	checknan("KRK^T", AS_1D(new->P), EKF_N * EKF_N);

	// I - KH
	static float IMKH[EKF_N][EKF_N];
	eyeN(AS_1D(IMKH), EKF_N);
	SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_M, -1.0, K, H, 1.0, IMKH);
	checknan("I-KH", AS_1D(IMKH), EKF_N * EKF_N);
	checknan("old->P", AS_1D(old->P), EKF_N * EKF_N);

	static float PIMKHt[EKF_N][EKF_N];
	ZEROARR(PIMKHt);
	SGEMM2D('n', 't', EKF_N, EKF_N, EKF_N, 1.0, old->P, IMKH, 0.0, PIMKHt);
	checknan("P(I-KH)^T", AS_1D(PIMKHt), EKF_N * EKF_N);

	// recall that new->P already contains KRK^T, and we use beta=1.0 to add in-place
	SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_N, 1.0, IMKH, PIMKHt, 1.0, new->P);
	checknan("P-New", AS_1D(new->P), EKF_N * EKF_N);
}


#ifdef EKFTEST

#include <assert.h>
#include <stdio.h>
#include <float.h>

bool close(float a, float b, float tol) { return fabs(a - b) < tol; }
bool vclose(struct vec a, struct vec b, float tol)
{
	return close(a.x, b.x, tol) && close(a.y, b.y, tol) && close(a.z, b.z, tol);
}
bool qclose(struct quat a, struct quat b, float tol)
{
	return vclose(a.v, b.v, tol) && close(a.w, b.w, tol);
}

void vprintln(char const *name, struct vec v)
{
	printf("%s = (%f, %f, %f)\n", name, v.x, v.y, v.z);
}
void qprintln(char const *name, struct quat q)
{
	printf("%s = (%f, %f, %f | %f)\n", name, q.v.x, q.v.y, q.v.z, q.w);
}

int main()
{
	// math tests
	{	// zero gyro -> no rotation
		struct quat q = qeye();
		struct quat q1 = quat_gyro_update(q, vzero(), 0.1);
		assert(qclose(q, q1, FLT_MIN));

		q = qaxisangle(mkvec(100,-2,3), 0.15);
		struct quat qi = qinv(q);
		struct quat qiq = qqmul(qi, q);
		assert(qclose(qiq, qeye(), 0.000001));
	}

	{ // full circle pitch - the very fine dt is needed for exactness
		struct quat q = qeye();
		int N = 100000;
		struct vec rpy = mkvec(0, 2 * M_PI / N, 0);
		for (int i = 0; i < N; ++i) {
			q = quat_gyro_update(q, rpy, 1.0 / N);
		}
		assert(qclose(q, qeye(), 0.0001)); // loose tolerance for dumb euler integration
	}

	{
		struct vec x = mkvec(1, 0, 0);
		struct vec y = mkvec(0, 1, 0);
		struct vec z = mkvec(0, 0, 1);
		struct vec rx;
		rx = qvrot(qaxisangle(z, radians(90)), x);
		assert(vclose(rx, y, 0.000001));
		rx = qvrot(qaxisangle(z, radians(180)), x);
		assert(vclose(rx, vneg(x), 0.000001));
		rx = qvrot(qaxisangle(z, radians(90)), y);
		assert(vclose(rx, vneg(x), 0.000001));

		struct vec v_arbitrary = mkvec(1, 2, -1.2);
		struct quat q_arbitrary = qaxisangle(v_arbitrary, radians(36));
		rx = x;
		for (int i = 0; i < 10; ++i) {
			rx = qvrot(q_arbitrary, rx);
		}
		assert(vclose(rx, x, 0.000001));
	}

	// system tests
	struct ekf a, b;
	struct ekf *front = &a, *back = &b;
	//back->acc_var = 0.0;
	//back->gyro_var = 0.0;
	//back->bias_acc_var = 0.0;
	//back->bias_gyro_var = 0.0;

	// acceleration only
	float zeros[4] = {0, 0, 0, 0};
	struct quat q = qeye();
	ekf_init(back,  zeros, zeros, (float *)&q);
	ekf_init(front, zeros, zeros, (float *)&q);
	float dt = 1.0 / 100.0;
	float acc[3] = {0, 0, 1 + GRAV};
	float gyro[3] = {0, 0, 0};
	for (int i = 0; i < 100; ++i) {
		ekf_imu(back, front, acc, gyro, dt);
		struct ekf *tmp = front; front = back; back = tmp; // swap
	}
	assert(vclose(back->vel, mkvec(0, 0, 1), 0.000001));

	float vicon_pos[] = {0, 0, 0};
	float vicon_quat[] = {0, 0, 0, 1};
	ekf_vicon(back, front, vicon_pos, vicon_quat);

	puts("All tests passed.");

	printf("sizeof vec: %lu\n", sizeof(struct vec));
	printf("sizeof quat: %lu\n", sizeof(struct quat));
	printf("sizeof mat33: %lu\n", sizeof(struct mat33));
}
#endif // EKFTEST

// TODO: write more tests
