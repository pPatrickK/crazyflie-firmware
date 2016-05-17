#include <math.h>
#include <float.h>
#include <stdbool.h> // bool
#include <string.h>  // memcpy 

#include "ekf.h"
#include "ekf_cov_init.h"
#include "cholsl.c"
#include "mexutil.h"

// NOTE: this code depends on a strong optimizing compiler
// because it passes and returns multi-word structs by value.


// TODO move to impl

void ekf_mat_identity(float m[EKF_N][EKF_N])
{
	fzero(AS_1D(m), EKF_N * EKF_N);
	for (int i = 0; i < EKF_N; ++i) {
		m[i][i] = 1;
	}
}

void ekf_set_block33(float m[EKF_N][EKF_N], int row, int col, struct mat33 const *block)
{
	float *blockptr = &m[row][col];
	set_block33(blockptr, EKF_N, block);
}

void ekf_mult_block33(float m[EKF_N][EKF_N], int row, int col, struct mat33 const *a, struct mat33 const *b)
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


void ekf_init(struct ekf *ekf, float const pos[3], float const vel[3], float const quat[4])
{
	ekf->pos = vloadf(pos);
	ekf->vel = vloadf(vel);
	ekf->quat = qloadf(quat);
	memcpy(ekf->P, ekf_cov_init, sizeof(ekf_cov_init));
}

#include "addQ.h"

void zeroEKF(float m[EKF_N][EKF_N]) {
	for (int i = 0; i < EKF_N; ++i) {
		for (int j = 0; j < EKF_N; ++j) {
			m[i][j] = 0;
		}
	}
}

void copyEKF(float const src[EKF_N][EKF_N], float dst[EKF_N][EKF_N]) {
	for (int i = 0; i < EKF_N; ++i) {
		for (int j = 0; j < EKF_N; ++j) {
			dst[i][j] = src[i][j];
		}
	}
}

// IT'S 2016 and GCC FUCKING SUCKS at OPTIMIZING my PASS-BY-VALUE 3x3 MATRIX LIBRARY!!!
// I'M FRUSTRATED!!!!!!!!!!!!!!!!!

struct mat33 aXplusbI(float a, struct mat33 const *X, float b)
{
	struct mat33 m;

	m.m[0][0] = a * X->m[0][0] + b;
	m.m[0][1] = a * X->m[0][1];
	m.m[0][2] = a * X->m[0][2];
	
	m.m[1][0] = a * X->m[1][0];
	m.m[1][1] = a * X->m[1][1] + b;
	m.m[1][2] = a * X->m[1][2];

	m.m[2][0] = a * X->m[2][0];
	m.m[2][1] = a * X->m[2][1];
	m.m[2][2] = a * X->m[2][2] + b;

	return m;
}

void dynamic_matrix(struct quat const q, struct vec const omega, struct vec const acc, float const dt, float F[EKF_N][EKF_N])
{

	float const dt_p2_2 = dt * dt * 0.5;
	float const dt_p3_6 = dt_p2_2 * dt / 3.0;
	float const dt_p4_24 = dt_p3_6 * dt * 0.25;
	//float const dt_p5_120 = dt_p4_24 * dt * 0.2;

	struct mat33 C_eq = quat2rotmat(qinv(q));
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

	//ekf_set_block33(F, 0, 3, eyescl(dt));
	F[0][3] = F[1][4] = F[2][5] = dt;

	ekf_mult_block33(F, 0, 6, &Ca3, &A);

	ekf_mult_block33(F, 3, 6, &Ca3, &FF);

	ekf_set_block33(F, 6, 6, &E);
}

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
	ekf->quat = qnormalized(qinv(quat_gyro_update(qinv(ekf_prev->quat), omega, dt)));

	// compute true acceleration
	//struct vec const acc_imu = vsub(float2vec(acc), ekf->bias_acc);
	struct vec const acc_imu = vloadf(acc);
    struct vec acc_world = qvrot(qinv(ekf->quat), acc_imu);
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


void ekf_imu_d(struct ekf const *ekf_prev, struct ekf *ekf, double const acc[3], double const gyro[3], float dt)
{
	float accf[3] = { acc[0], acc[1], acc[2] };
	float gyrof[3] = { gyro[0], gyro[1], gyro[2] };
	ekf_imu(ekf_prev, ekf, accf, gyrof, dt);
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

/*
// TODO timestamp, buffer search
void ekf_vicon_old(struct ekf *ekf, float pos[3], float quat[4])
{
	struct vec pos_vicon = float2vec(pos);
	struct quat quat_vicon = mkquat(quat[0], quat[1], quat[2], quat[3]);

	// error quaternion
	struct quat q_residual = qqmul(qinv(ekf->quat), quat_vicon);
	struct vec err_quat = vdiv(q_residual.v, (2 * q_residual.w));

	// construct residual - will cast to float[6] later
	struct vec residual[2];
	residual[0] = vsub(pos_vicon, ekf->pos);
	residual[1] = err_quat;
	checknan("residual", (float const *)&residual, 6);

	// S = H P H' + R  :  innovation
	// H is constant
	SGEMM2D('n', 't', EKF_N, EKF_M, EKF_N, 1.0, ekf->P, ekf->H, 0.0, ekf->temp);
	SGEMM2D('n', 'n', EKF_M, EKF_M, EKF_N, 1.0, ekf->H, ekf->temp, 0.0, ekf->S);
	for (int i = 0; i < EKF_M; ++i) { ekf->S[i][i] += ekf->R[i][i]; } // diag only, no cov
	checknan("S", AS_1D(ekf->S), EKF_M * EKF_M);

	// K = P H' S^-1   :  gain
	float scratch[EKF_M];
	cholsl(AS_1D(ekf->S), AS_1D(ekf->temp), scratch, EKF_M);
	checknan("S^1", AS_1D(ekf->temp), EKF_M * EKF_M);
	SGEMM2D('t', 'n', EKF_N, EKF_M, EKF_M, 1.0, ekf->H, ekf->temp, 0.0, ekf->temp2);
	checknan("temp2", AS_1D(ekf->temp2), EKF_N * EKF_M);
	SGEMM2D('n', 'n', EKF_N, EKF_M, EKF_N, 1.0, ekf->P, ekf->temp2, 0.0, ekf->K);
	checknan("K", AS_1D(ekf->K), EKF_N * EKF_M);

	// compute and apply correction
	float correction[EKF_N];
	sgemm('n', 'n', EKF_N, 1, EKF_M, 1.0, AS_1D(ekf->K), (float const *)&residual, 0.0, correction);
	vaddi(&ekf->pos, float2vec(correction + 0));
	vaddi(&ekf->vel, float2vec(correction + 3));
	struct quat error_quat = qrpy_small(float2vec(correction + 6));
	ekf->quat = qnormalized(qqmul(ekf->quat, error_quat));
	vaddi(&ekf->bias_gyro, float2vec(correction + 9));
	vaddi(&ekf->bias_acc, float2vec(correction + 12));

	// covariance update
	// Pnew = (I - KH) P (I - KH)^T + KRK^T
	//SGEMM2D('n', 't', EKF_M, EKF_N, EKF_M, 1.0, ekf->R, ekf->K, 0.0, ekf->temp2);
	//SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_M, 1.0, ekf->K, ekf->temp2, 0.0, ekf->P);
	//eyeN(AS_1D(ekf->temp), EKF_N);
	//SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_M, -1.0, ekf->K, ekf->H, 1.0, ekf->temp);
	//SGEMM2D('n', 't', EKF_N, EKF_N, EKF_N, 1.0, ekf->P, ekf->temp, 0.0, ekf->temp2);
	//SGEMM2D('n', 'n', EKF_N, EKF_N, EKF_N, 1.0, ekf->temp, ekf->temp2, 1.0, ekf->P);
}
*/

void ekf_vicon_d(struct ekf const *old, struct ekf *new, double const pos[3], double const quat[4])//, double *debug)
{
	float posf[] = { pos[0], pos[1], pos[2] };
	float quatf[] = { quat[0], quat[1], quat[2], quat[3] };
	ekf_vicon(old, new, posf, quatf);
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
