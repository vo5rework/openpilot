#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2768303435411887346);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1488408519072955526);
void pose_H_mod_fun(double *state, double *out_1747102118856192496);
void pose_f_fun(double *state, double dt, double *out_7088515702411044915);
void pose_F_fun(double *state, double dt, double *out_1598216342666213795);
void pose_h_4(double *state, double *unused, double *out_5322908657184251983);
void pose_H_4(double *state, double *unused, double *out_2501263120910905103);
void pose_h_10(double *state, double *unused, double *out_3029307404736649758);
void pose_H_10(double *state, double *unused, double *out_3781124612136597110);
void pose_h_13(double *state, double *unused, double *out_1688939920546120950);
void pose_H_13(double *state, double *unused, double *out_8334849744481945584);
void pose_h_14(double *state, double *unused, double *out_6837219853172441902);
void pose_H_14(double *state, double *unused, double *out_581525311384467193);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}