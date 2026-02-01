#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8591711695998480772);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8158935169591926375);
void car_H_mod_fun(double *state, double *out_3133600418890784736);
void car_f_fun(double *state, double dt, double *out_2332475247318632001);
void car_F_fun(double *state, double dt, double *out_1574125402589401427);
void car_h_25(double *state, double *unused, double *out_5037007449914352563);
void car_H_25(double *state, double *unused, double *out_8037292244075449576);
void car_h_24(double *state, double *unused, double *out_6233203168910131630);
void car_H_24(double *state, double *unused, double *out_8232237406026952067);
void car_h_30(double *state, double *unused, double *out_5528463973041149868);
void car_H_30(double *state, double *unused, double *out_7891118871126853413);
void car_h_26(double *state, double *unused, double *out_7916606833007556103);
void car_H_26(double *state, double *unused, double *out_4295788925201393352);
void car_h_27(double *state, double *unused, double *out_298861400407538633);
void car_H_27(double *state, double *unused, double *out_5667524799942910196);
void car_h_29(double *state, double *unused, double *out_3936310928661560983);
void car_H_29(double *state, double *unused, double *out_7380887526812461229);
void car_h_28(double *state, double *unused, double *out_4696813481689167645);
void car_H_28(double *state, double *unused, double *out_5983457529827559813);
void car_h_31(double *state, double *unused, double *out_2836219441392886127);
void car_H_31(double *state, double *unused, double *out_8067938205952410004);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}