#pragma once

// algorithm to compute variance online, from Knuth / Welford
// avoids using up kilobytes of memory for computing stats of many data points
//
// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm

#define STAT_DIM 3

struct online_stats
{
	int count;
	float mean[STAT_DIM];
	float M2[STAT_DIM];
};

void init_online_stats(struct online_stats *state)
{
	state->count = 0;
	for (int i = 0; i < STAT_DIM; ++i) {
		state->mean[i] = 0.0f;
		state->M2[i] = 0.0f;
	}
}

void update_online_stats(struct online_stats *state, float const sample[STAT_DIM])
{
	state->n++;
	for (int i = 0; i < STAT_DIM; ++i) {
		float x = sample[i];
		float delta = x - state->mean[i];
		state->mean[i] += delta / state->n;
		state->M2[i] += delta * (x - state->mean[i]);
	}
}

void get_online_stats(struct online_stats const *state, float mean[STAT_DIM], float var[STAT_DIM])
{
	for (int i = 0; i < STAT_DIM; ++i) {
		var[i] = state->M2[i] / (state->n - 1);
		mean[i] = state->mean[i];
	}
}

