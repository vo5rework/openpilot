#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1886515091563246719);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8728219793499801644);
void pose_H_mod_fun(double *state, double *out_5583056958182896189);
void pose_f_fun(double *state, double dt, double *out_6878832699624430321);
void pose_F_fun(double *state, double dt, double *out_2457315738183324621);
void pose_h_4(double *state, double *unused, double *out_7759867018497998577);
void pose_H_4(double *state, double *unused, double *out_3361304707874925153);
void pose_h_10(double *state, double *unused, double *out_5255984674989194447);
void pose_H_10(double *state, double *unused, double *out_9077457845341768252);
void pose_h_13(double *state, double *unused, double *out_5774649359282809780);
void pose_H_13(double *state, double *unused, double *out_149030882542592352);
void pose_h_14(double *state, double *unused, double *out_5470742825461452978);
void pose_H_14(double *state, double *unused, double *out_601936148464559376);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}