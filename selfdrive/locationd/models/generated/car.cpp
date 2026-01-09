#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8852059517079413622) {
   out_8852059517079413622[0] = delta_x[0] + nom_x[0];
   out_8852059517079413622[1] = delta_x[1] + nom_x[1];
   out_8852059517079413622[2] = delta_x[2] + nom_x[2];
   out_8852059517079413622[3] = delta_x[3] + nom_x[3];
   out_8852059517079413622[4] = delta_x[4] + nom_x[4];
   out_8852059517079413622[5] = delta_x[5] + nom_x[5];
   out_8852059517079413622[6] = delta_x[6] + nom_x[6];
   out_8852059517079413622[7] = delta_x[7] + nom_x[7];
   out_8852059517079413622[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_406763681668113800) {
   out_406763681668113800[0] = -nom_x[0] + true_x[0];
   out_406763681668113800[1] = -nom_x[1] + true_x[1];
   out_406763681668113800[2] = -nom_x[2] + true_x[2];
   out_406763681668113800[3] = -nom_x[3] + true_x[3];
   out_406763681668113800[4] = -nom_x[4] + true_x[4];
   out_406763681668113800[5] = -nom_x[5] + true_x[5];
   out_406763681668113800[6] = -nom_x[6] + true_x[6];
   out_406763681668113800[7] = -nom_x[7] + true_x[7];
   out_406763681668113800[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3891528276650776736) {
   out_3891528276650776736[0] = 1.0;
   out_3891528276650776736[1] = 0.0;
   out_3891528276650776736[2] = 0.0;
   out_3891528276650776736[3] = 0.0;
   out_3891528276650776736[4] = 0.0;
   out_3891528276650776736[5] = 0.0;
   out_3891528276650776736[6] = 0.0;
   out_3891528276650776736[7] = 0.0;
   out_3891528276650776736[8] = 0.0;
   out_3891528276650776736[9] = 0.0;
   out_3891528276650776736[10] = 1.0;
   out_3891528276650776736[11] = 0.0;
   out_3891528276650776736[12] = 0.0;
   out_3891528276650776736[13] = 0.0;
   out_3891528276650776736[14] = 0.0;
   out_3891528276650776736[15] = 0.0;
   out_3891528276650776736[16] = 0.0;
   out_3891528276650776736[17] = 0.0;
   out_3891528276650776736[18] = 0.0;
   out_3891528276650776736[19] = 0.0;
   out_3891528276650776736[20] = 1.0;
   out_3891528276650776736[21] = 0.0;
   out_3891528276650776736[22] = 0.0;
   out_3891528276650776736[23] = 0.0;
   out_3891528276650776736[24] = 0.0;
   out_3891528276650776736[25] = 0.0;
   out_3891528276650776736[26] = 0.0;
   out_3891528276650776736[27] = 0.0;
   out_3891528276650776736[28] = 0.0;
   out_3891528276650776736[29] = 0.0;
   out_3891528276650776736[30] = 1.0;
   out_3891528276650776736[31] = 0.0;
   out_3891528276650776736[32] = 0.0;
   out_3891528276650776736[33] = 0.0;
   out_3891528276650776736[34] = 0.0;
   out_3891528276650776736[35] = 0.0;
   out_3891528276650776736[36] = 0.0;
   out_3891528276650776736[37] = 0.0;
   out_3891528276650776736[38] = 0.0;
   out_3891528276650776736[39] = 0.0;
   out_3891528276650776736[40] = 1.0;
   out_3891528276650776736[41] = 0.0;
   out_3891528276650776736[42] = 0.0;
   out_3891528276650776736[43] = 0.0;
   out_3891528276650776736[44] = 0.0;
   out_3891528276650776736[45] = 0.0;
   out_3891528276650776736[46] = 0.0;
   out_3891528276650776736[47] = 0.0;
   out_3891528276650776736[48] = 0.0;
   out_3891528276650776736[49] = 0.0;
   out_3891528276650776736[50] = 1.0;
   out_3891528276650776736[51] = 0.0;
   out_3891528276650776736[52] = 0.0;
   out_3891528276650776736[53] = 0.0;
   out_3891528276650776736[54] = 0.0;
   out_3891528276650776736[55] = 0.0;
   out_3891528276650776736[56] = 0.0;
   out_3891528276650776736[57] = 0.0;
   out_3891528276650776736[58] = 0.0;
   out_3891528276650776736[59] = 0.0;
   out_3891528276650776736[60] = 1.0;
   out_3891528276650776736[61] = 0.0;
   out_3891528276650776736[62] = 0.0;
   out_3891528276650776736[63] = 0.0;
   out_3891528276650776736[64] = 0.0;
   out_3891528276650776736[65] = 0.0;
   out_3891528276650776736[66] = 0.0;
   out_3891528276650776736[67] = 0.0;
   out_3891528276650776736[68] = 0.0;
   out_3891528276650776736[69] = 0.0;
   out_3891528276650776736[70] = 1.0;
   out_3891528276650776736[71] = 0.0;
   out_3891528276650776736[72] = 0.0;
   out_3891528276650776736[73] = 0.0;
   out_3891528276650776736[74] = 0.0;
   out_3891528276650776736[75] = 0.0;
   out_3891528276650776736[76] = 0.0;
   out_3891528276650776736[77] = 0.0;
   out_3891528276650776736[78] = 0.0;
   out_3891528276650776736[79] = 0.0;
   out_3891528276650776736[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4356035449915671017) {
   out_4356035449915671017[0] = state[0];
   out_4356035449915671017[1] = state[1];
   out_4356035449915671017[2] = state[2];
   out_4356035449915671017[3] = state[3];
   out_4356035449915671017[4] = state[4];
   out_4356035449915671017[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4356035449915671017[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4356035449915671017[7] = state[7];
   out_4356035449915671017[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3255253793337990890) {
   out_3255253793337990890[0] = 1;
   out_3255253793337990890[1] = 0;
   out_3255253793337990890[2] = 0;
   out_3255253793337990890[3] = 0;
   out_3255253793337990890[4] = 0;
   out_3255253793337990890[5] = 0;
   out_3255253793337990890[6] = 0;
   out_3255253793337990890[7] = 0;
   out_3255253793337990890[8] = 0;
   out_3255253793337990890[9] = 0;
   out_3255253793337990890[10] = 1;
   out_3255253793337990890[11] = 0;
   out_3255253793337990890[12] = 0;
   out_3255253793337990890[13] = 0;
   out_3255253793337990890[14] = 0;
   out_3255253793337990890[15] = 0;
   out_3255253793337990890[16] = 0;
   out_3255253793337990890[17] = 0;
   out_3255253793337990890[18] = 0;
   out_3255253793337990890[19] = 0;
   out_3255253793337990890[20] = 1;
   out_3255253793337990890[21] = 0;
   out_3255253793337990890[22] = 0;
   out_3255253793337990890[23] = 0;
   out_3255253793337990890[24] = 0;
   out_3255253793337990890[25] = 0;
   out_3255253793337990890[26] = 0;
   out_3255253793337990890[27] = 0;
   out_3255253793337990890[28] = 0;
   out_3255253793337990890[29] = 0;
   out_3255253793337990890[30] = 1;
   out_3255253793337990890[31] = 0;
   out_3255253793337990890[32] = 0;
   out_3255253793337990890[33] = 0;
   out_3255253793337990890[34] = 0;
   out_3255253793337990890[35] = 0;
   out_3255253793337990890[36] = 0;
   out_3255253793337990890[37] = 0;
   out_3255253793337990890[38] = 0;
   out_3255253793337990890[39] = 0;
   out_3255253793337990890[40] = 1;
   out_3255253793337990890[41] = 0;
   out_3255253793337990890[42] = 0;
   out_3255253793337990890[43] = 0;
   out_3255253793337990890[44] = 0;
   out_3255253793337990890[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3255253793337990890[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3255253793337990890[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3255253793337990890[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3255253793337990890[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3255253793337990890[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3255253793337990890[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3255253793337990890[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3255253793337990890[53] = -9.8100000000000005*dt;
   out_3255253793337990890[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3255253793337990890[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3255253793337990890[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3255253793337990890[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3255253793337990890[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3255253793337990890[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3255253793337990890[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3255253793337990890[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3255253793337990890[62] = 0;
   out_3255253793337990890[63] = 0;
   out_3255253793337990890[64] = 0;
   out_3255253793337990890[65] = 0;
   out_3255253793337990890[66] = 0;
   out_3255253793337990890[67] = 0;
   out_3255253793337990890[68] = 0;
   out_3255253793337990890[69] = 0;
   out_3255253793337990890[70] = 1;
   out_3255253793337990890[71] = 0;
   out_3255253793337990890[72] = 0;
   out_3255253793337990890[73] = 0;
   out_3255253793337990890[74] = 0;
   out_3255253793337990890[75] = 0;
   out_3255253793337990890[76] = 0;
   out_3255253793337990890[77] = 0;
   out_3255253793337990890[78] = 0;
   out_3255253793337990890[79] = 0;
   out_3255253793337990890[80] = 1;
}
void h_25(double *state, double *unused, double *out_7088141680002210025) {
   out_7088141680002210025[0] = state[6];
}
void H_25(double *state, double *unused, double *out_368479638005411588) {
   out_368479638005411588[0] = 0;
   out_368479638005411588[1] = 0;
   out_368479638005411588[2] = 0;
   out_368479638005411588[3] = 0;
   out_368479638005411588[4] = 0;
   out_368479638005411588[5] = 0;
   out_368479638005411588[6] = 1;
   out_368479638005411588[7] = 0;
   out_368479638005411588[8] = 0;
}
void h_24(double *state, double *unused, double *out_1845441017641887135) {
   out_1845441017641887135[0] = state[4];
   out_1845441017641887135[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1352545508846725256) {
   out_1352545508846725256[0] = 0;
   out_1352545508846725256[1] = 0;
   out_1352545508846725256[2] = 0;
   out_1352545508846725256[3] = 0;
   out_1352545508846725256[4] = 1;
   out_1352545508846725256[5] = 0;
   out_1352545508846725256[6] = 0;
   out_1352545508846725256[7] = 0;
   out_1352545508846725256[8] = 0;
   out_1352545508846725256[9] = 0;
   out_1352545508846725256[10] = 0;
   out_1352545508846725256[11] = 0;
   out_1352545508846725256[12] = 0;
   out_1352545508846725256[13] = 0;
   out_1352545508846725256[14] = 1;
   out_1352545508846725256[15] = 0;
   out_1352545508846725256[16] = 0;
   out_1352545508846725256[17] = 0;
}
void h_30(double *state, double *unused, double *out_8603517323688590270) {
   out_8603517323688590270[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2886812596512660215) {
   out_2886812596512660215[0] = 0;
   out_2886812596512660215[1] = 0;
   out_2886812596512660215[2] = 0;
   out_2886812596512660215[3] = 0;
   out_2886812596512660215[4] = 1;
   out_2886812596512660215[5] = 0;
   out_2886812596512660215[6] = 0;
   out_2886812596512660215[7] = 0;
   out_2886812596512660215[8] = 0;
}
void h_26(double *state, double *unused, double *out_4841553482360115701) {
   out_4841553482360115701[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3673005607766212189) {
   out_3673005607766212189[0] = 0;
   out_3673005607766212189[1] = 0;
   out_3673005607766212189[2] = 0;
   out_3673005607766212189[3] = 0;
   out_3673005607766212189[4] = 0;
   out_3673005607766212189[5] = 0;
   out_3673005607766212189[6] = 0;
   out_3673005607766212189[7] = 1;
   out_3673005607766212189[8] = 0;
}
void h_27(double *state, double *unused, double *out_2086468412341364380) {
   out_2086468412341364380[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5110406667696603432) {
   out_5110406667696603432[0] = 0;
   out_5110406667696603432[1] = 0;
   out_5110406667696603432[2] = 0;
   out_5110406667696603432[3] = 1;
   out_5110406667696603432[4] = 0;
   out_5110406667696603432[5] = 0;
   out_5110406667696603432[6] = 0;
   out_5110406667696603432[7] = 0;
   out_5110406667696603432[8] = 0;
}
void h_29(double *state, double *unused, double *out_8899716897119298911) {
   out_8899716897119298911[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3397043940827052399) {
   out_3397043940827052399[0] = 0;
   out_3397043940827052399[1] = 1;
   out_3397043940827052399[2] = 0;
   out_3397043940827052399[3] = 0;
   out_3397043940827052399[4] = 0;
   out_3397043940827052399[5] = 0;
   out_3397043940827052399[6] = 0;
   out_3397043940827052399[7] = 0;
   out_3397043940827052399[8] = 0;
}
void h_28(double *state, double *unused, double *out_4690873733150062031) {
   out_4690873733150062031[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1685355076242478175) {
   out_1685355076242478175[0] = 1;
   out_1685355076242478175[1] = 0;
   out_1685355076242478175[2] = 0;
   out_1685355076242478175[3] = 0;
   out_1685355076242478175[4] = 0;
   out_1685355076242478175[5] = 0;
   out_1685355076242478175[6] = 0;
   out_1685355076242478175[7] = 0;
   out_1685355076242478175[8] = 0;
}
void h_31(double *state, double *unused, double *out_7721152865453319241) {
   out_7721152865453319241[0] = state[8];
}
void H_31(double *state, double *unused, double *out_399125599882372016) {
   out_399125599882372016[0] = 0;
   out_399125599882372016[1] = 0;
   out_399125599882372016[2] = 0;
   out_399125599882372016[3] = 0;
   out_399125599882372016[4] = 0;
   out_399125599882372016[5] = 0;
   out_399125599882372016[6] = 0;
   out_399125599882372016[7] = 0;
   out_399125599882372016[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_8852059517079413622) {
  err_fun(nom_x, delta_x, out_8852059517079413622);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_406763681668113800) {
  inv_err_fun(nom_x, true_x, out_406763681668113800);
}
void car_H_mod_fun(double *state, double *out_3891528276650776736) {
  H_mod_fun(state, out_3891528276650776736);
}
void car_f_fun(double *state, double dt, double *out_4356035449915671017) {
  f_fun(state,  dt, out_4356035449915671017);
}
void car_F_fun(double *state, double dt, double *out_3255253793337990890) {
  F_fun(state,  dt, out_3255253793337990890);
}
void car_h_25(double *state, double *unused, double *out_7088141680002210025) {
  h_25(state, unused, out_7088141680002210025);
}
void car_H_25(double *state, double *unused, double *out_368479638005411588) {
  H_25(state, unused, out_368479638005411588);
}
void car_h_24(double *state, double *unused, double *out_1845441017641887135) {
  h_24(state, unused, out_1845441017641887135);
}
void car_H_24(double *state, double *unused, double *out_1352545508846725256) {
  H_24(state, unused, out_1352545508846725256);
}
void car_h_30(double *state, double *unused, double *out_8603517323688590270) {
  h_30(state, unused, out_8603517323688590270);
}
void car_H_30(double *state, double *unused, double *out_2886812596512660215) {
  H_30(state, unused, out_2886812596512660215);
}
void car_h_26(double *state, double *unused, double *out_4841553482360115701) {
  h_26(state, unused, out_4841553482360115701);
}
void car_H_26(double *state, double *unused, double *out_3673005607766212189) {
  H_26(state, unused, out_3673005607766212189);
}
void car_h_27(double *state, double *unused, double *out_2086468412341364380) {
  h_27(state, unused, out_2086468412341364380);
}
void car_H_27(double *state, double *unused, double *out_5110406667696603432) {
  H_27(state, unused, out_5110406667696603432);
}
void car_h_29(double *state, double *unused, double *out_8899716897119298911) {
  h_29(state, unused, out_8899716897119298911);
}
void car_H_29(double *state, double *unused, double *out_3397043940827052399) {
  H_29(state, unused, out_3397043940827052399);
}
void car_h_28(double *state, double *unused, double *out_4690873733150062031) {
  h_28(state, unused, out_4690873733150062031);
}
void car_H_28(double *state, double *unused, double *out_1685355076242478175) {
  H_28(state, unused, out_1685355076242478175);
}
void car_h_31(double *state, double *unused, double *out_7721152865453319241) {
  h_31(state, unused, out_7721152865453319241);
}
void car_H_31(double *state, double *unused, double *out_399125599882372016) {
  H_31(state, unused, out_399125599882372016);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
