#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1886515091563246719) {
   out_1886515091563246719[0] = delta_x[0] + nom_x[0];
   out_1886515091563246719[1] = delta_x[1] + nom_x[1];
   out_1886515091563246719[2] = delta_x[2] + nom_x[2];
   out_1886515091563246719[3] = delta_x[3] + nom_x[3];
   out_1886515091563246719[4] = delta_x[4] + nom_x[4];
   out_1886515091563246719[5] = delta_x[5] + nom_x[5];
   out_1886515091563246719[6] = delta_x[6] + nom_x[6];
   out_1886515091563246719[7] = delta_x[7] + nom_x[7];
   out_1886515091563246719[8] = delta_x[8] + nom_x[8];
   out_1886515091563246719[9] = delta_x[9] + nom_x[9];
   out_1886515091563246719[10] = delta_x[10] + nom_x[10];
   out_1886515091563246719[11] = delta_x[11] + nom_x[11];
   out_1886515091563246719[12] = delta_x[12] + nom_x[12];
   out_1886515091563246719[13] = delta_x[13] + nom_x[13];
   out_1886515091563246719[14] = delta_x[14] + nom_x[14];
   out_1886515091563246719[15] = delta_x[15] + nom_x[15];
   out_1886515091563246719[16] = delta_x[16] + nom_x[16];
   out_1886515091563246719[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8728219793499801644) {
   out_8728219793499801644[0] = -nom_x[0] + true_x[0];
   out_8728219793499801644[1] = -nom_x[1] + true_x[1];
   out_8728219793499801644[2] = -nom_x[2] + true_x[2];
   out_8728219793499801644[3] = -nom_x[3] + true_x[3];
   out_8728219793499801644[4] = -nom_x[4] + true_x[4];
   out_8728219793499801644[5] = -nom_x[5] + true_x[5];
   out_8728219793499801644[6] = -nom_x[6] + true_x[6];
   out_8728219793499801644[7] = -nom_x[7] + true_x[7];
   out_8728219793499801644[8] = -nom_x[8] + true_x[8];
   out_8728219793499801644[9] = -nom_x[9] + true_x[9];
   out_8728219793499801644[10] = -nom_x[10] + true_x[10];
   out_8728219793499801644[11] = -nom_x[11] + true_x[11];
   out_8728219793499801644[12] = -nom_x[12] + true_x[12];
   out_8728219793499801644[13] = -nom_x[13] + true_x[13];
   out_8728219793499801644[14] = -nom_x[14] + true_x[14];
   out_8728219793499801644[15] = -nom_x[15] + true_x[15];
   out_8728219793499801644[16] = -nom_x[16] + true_x[16];
   out_8728219793499801644[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5583056958182896189) {
   out_5583056958182896189[0] = 1.0;
   out_5583056958182896189[1] = 0.0;
   out_5583056958182896189[2] = 0.0;
   out_5583056958182896189[3] = 0.0;
   out_5583056958182896189[4] = 0.0;
   out_5583056958182896189[5] = 0.0;
   out_5583056958182896189[6] = 0.0;
   out_5583056958182896189[7] = 0.0;
   out_5583056958182896189[8] = 0.0;
   out_5583056958182896189[9] = 0.0;
   out_5583056958182896189[10] = 0.0;
   out_5583056958182896189[11] = 0.0;
   out_5583056958182896189[12] = 0.0;
   out_5583056958182896189[13] = 0.0;
   out_5583056958182896189[14] = 0.0;
   out_5583056958182896189[15] = 0.0;
   out_5583056958182896189[16] = 0.0;
   out_5583056958182896189[17] = 0.0;
   out_5583056958182896189[18] = 0.0;
   out_5583056958182896189[19] = 1.0;
   out_5583056958182896189[20] = 0.0;
   out_5583056958182896189[21] = 0.0;
   out_5583056958182896189[22] = 0.0;
   out_5583056958182896189[23] = 0.0;
   out_5583056958182896189[24] = 0.0;
   out_5583056958182896189[25] = 0.0;
   out_5583056958182896189[26] = 0.0;
   out_5583056958182896189[27] = 0.0;
   out_5583056958182896189[28] = 0.0;
   out_5583056958182896189[29] = 0.0;
   out_5583056958182896189[30] = 0.0;
   out_5583056958182896189[31] = 0.0;
   out_5583056958182896189[32] = 0.0;
   out_5583056958182896189[33] = 0.0;
   out_5583056958182896189[34] = 0.0;
   out_5583056958182896189[35] = 0.0;
   out_5583056958182896189[36] = 0.0;
   out_5583056958182896189[37] = 0.0;
   out_5583056958182896189[38] = 1.0;
   out_5583056958182896189[39] = 0.0;
   out_5583056958182896189[40] = 0.0;
   out_5583056958182896189[41] = 0.0;
   out_5583056958182896189[42] = 0.0;
   out_5583056958182896189[43] = 0.0;
   out_5583056958182896189[44] = 0.0;
   out_5583056958182896189[45] = 0.0;
   out_5583056958182896189[46] = 0.0;
   out_5583056958182896189[47] = 0.0;
   out_5583056958182896189[48] = 0.0;
   out_5583056958182896189[49] = 0.0;
   out_5583056958182896189[50] = 0.0;
   out_5583056958182896189[51] = 0.0;
   out_5583056958182896189[52] = 0.0;
   out_5583056958182896189[53] = 0.0;
   out_5583056958182896189[54] = 0.0;
   out_5583056958182896189[55] = 0.0;
   out_5583056958182896189[56] = 0.0;
   out_5583056958182896189[57] = 1.0;
   out_5583056958182896189[58] = 0.0;
   out_5583056958182896189[59] = 0.0;
   out_5583056958182896189[60] = 0.0;
   out_5583056958182896189[61] = 0.0;
   out_5583056958182896189[62] = 0.0;
   out_5583056958182896189[63] = 0.0;
   out_5583056958182896189[64] = 0.0;
   out_5583056958182896189[65] = 0.0;
   out_5583056958182896189[66] = 0.0;
   out_5583056958182896189[67] = 0.0;
   out_5583056958182896189[68] = 0.0;
   out_5583056958182896189[69] = 0.0;
   out_5583056958182896189[70] = 0.0;
   out_5583056958182896189[71] = 0.0;
   out_5583056958182896189[72] = 0.0;
   out_5583056958182896189[73] = 0.0;
   out_5583056958182896189[74] = 0.0;
   out_5583056958182896189[75] = 0.0;
   out_5583056958182896189[76] = 1.0;
   out_5583056958182896189[77] = 0.0;
   out_5583056958182896189[78] = 0.0;
   out_5583056958182896189[79] = 0.0;
   out_5583056958182896189[80] = 0.0;
   out_5583056958182896189[81] = 0.0;
   out_5583056958182896189[82] = 0.0;
   out_5583056958182896189[83] = 0.0;
   out_5583056958182896189[84] = 0.0;
   out_5583056958182896189[85] = 0.0;
   out_5583056958182896189[86] = 0.0;
   out_5583056958182896189[87] = 0.0;
   out_5583056958182896189[88] = 0.0;
   out_5583056958182896189[89] = 0.0;
   out_5583056958182896189[90] = 0.0;
   out_5583056958182896189[91] = 0.0;
   out_5583056958182896189[92] = 0.0;
   out_5583056958182896189[93] = 0.0;
   out_5583056958182896189[94] = 0.0;
   out_5583056958182896189[95] = 1.0;
   out_5583056958182896189[96] = 0.0;
   out_5583056958182896189[97] = 0.0;
   out_5583056958182896189[98] = 0.0;
   out_5583056958182896189[99] = 0.0;
   out_5583056958182896189[100] = 0.0;
   out_5583056958182896189[101] = 0.0;
   out_5583056958182896189[102] = 0.0;
   out_5583056958182896189[103] = 0.0;
   out_5583056958182896189[104] = 0.0;
   out_5583056958182896189[105] = 0.0;
   out_5583056958182896189[106] = 0.0;
   out_5583056958182896189[107] = 0.0;
   out_5583056958182896189[108] = 0.0;
   out_5583056958182896189[109] = 0.0;
   out_5583056958182896189[110] = 0.0;
   out_5583056958182896189[111] = 0.0;
   out_5583056958182896189[112] = 0.0;
   out_5583056958182896189[113] = 0.0;
   out_5583056958182896189[114] = 1.0;
   out_5583056958182896189[115] = 0.0;
   out_5583056958182896189[116] = 0.0;
   out_5583056958182896189[117] = 0.0;
   out_5583056958182896189[118] = 0.0;
   out_5583056958182896189[119] = 0.0;
   out_5583056958182896189[120] = 0.0;
   out_5583056958182896189[121] = 0.0;
   out_5583056958182896189[122] = 0.0;
   out_5583056958182896189[123] = 0.0;
   out_5583056958182896189[124] = 0.0;
   out_5583056958182896189[125] = 0.0;
   out_5583056958182896189[126] = 0.0;
   out_5583056958182896189[127] = 0.0;
   out_5583056958182896189[128] = 0.0;
   out_5583056958182896189[129] = 0.0;
   out_5583056958182896189[130] = 0.0;
   out_5583056958182896189[131] = 0.0;
   out_5583056958182896189[132] = 0.0;
   out_5583056958182896189[133] = 1.0;
   out_5583056958182896189[134] = 0.0;
   out_5583056958182896189[135] = 0.0;
   out_5583056958182896189[136] = 0.0;
   out_5583056958182896189[137] = 0.0;
   out_5583056958182896189[138] = 0.0;
   out_5583056958182896189[139] = 0.0;
   out_5583056958182896189[140] = 0.0;
   out_5583056958182896189[141] = 0.0;
   out_5583056958182896189[142] = 0.0;
   out_5583056958182896189[143] = 0.0;
   out_5583056958182896189[144] = 0.0;
   out_5583056958182896189[145] = 0.0;
   out_5583056958182896189[146] = 0.0;
   out_5583056958182896189[147] = 0.0;
   out_5583056958182896189[148] = 0.0;
   out_5583056958182896189[149] = 0.0;
   out_5583056958182896189[150] = 0.0;
   out_5583056958182896189[151] = 0.0;
   out_5583056958182896189[152] = 1.0;
   out_5583056958182896189[153] = 0.0;
   out_5583056958182896189[154] = 0.0;
   out_5583056958182896189[155] = 0.0;
   out_5583056958182896189[156] = 0.0;
   out_5583056958182896189[157] = 0.0;
   out_5583056958182896189[158] = 0.0;
   out_5583056958182896189[159] = 0.0;
   out_5583056958182896189[160] = 0.0;
   out_5583056958182896189[161] = 0.0;
   out_5583056958182896189[162] = 0.0;
   out_5583056958182896189[163] = 0.0;
   out_5583056958182896189[164] = 0.0;
   out_5583056958182896189[165] = 0.0;
   out_5583056958182896189[166] = 0.0;
   out_5583056958182896189[167] = 0.0;
   out_5583056958182896189[168] = 0.0;
   out_5583056958182896189[169] = 0.0;
   out_5583056958182896189[170] = 0.0;
   out_5583056958182896189[171] = 1.0;
   out_5583056958182896189[172] = 0.0;
   out_5583056958182896189[173] = 0.0;
   out_5583056958182896189[174] = 0.0;
   out_5583056958182896189[175] = 0.0;
   out_5583056958182896189[176] = 0.0;
   out_5583056958182896189[177] = 0.0;
   out_5583056958182896189[178] = 0.0;
   out_5583056958182896189[179] = 0.0;
   out_5583056958182896189[180] = 0.0;
   out_5583056958182896189[181] = 0.0;
   out_5583056958182896189[182] = 0.0;
   out_5583056958182896189[183] = 0.0;
   out_5583056958182896189[184] = 0.0;
   out_5583056958182896189[185] = 0.0;
   out_5583056958182896189[186] = 0.0;
   out_5583056958182896189[187] = 0.0;
   out_5583056958182896189[188] = 0.0;
   out_5583056958182896189[189] = 0.0;
   out_5583056958182896189[190] = 1.0;
   out_5583056958182896189[191] = 0.0;
   out_5583056958182896189[192] = 0.0;
   out_5583056958182896189[193] = 0.0;
   out_5583056958182896189[194] = 0.0;
   out_5583056958182896189[195] = 0.0;
   out_5583056958182896189[196] = 0.0;
   out_5583056958182896189[197] = 0.0;
   out_5583056958182896189[198] = 0.0;
   out_5583056958182896189[199] = 0.0;
   out_5583056958182896189[200] = 0.0;
   out_5583056958182896189[201] = 0.0;
   out_5583056958182896189[202] = 0.0;
   out_5583056958182896189[203] = 0.0;
   out_5583056958182896189[204] = 0.0;
   out_5583056958182896189[205] = 0.0;
   out_5583056958182896189[206] = 0.0;
   out_5583056958182896189[207] = 0.0;
   out_5583056958182896189[208] = 0.0;
   out_5583056958182896189[209] = 1.0;
   out_5583056958182896189[210] = 0.0;
   out_5583056958182896189[211] = 0.0;
   out_5583056958182896189[212] = 0.0;
   out_5583056958182896189[213] = 0.0;
   out_5583056958182896189[214] = 0.0;
   out_5583056958182896189[215] = 0.0;
   out_5583056958182896189[216] = 0.0;
   out_5583056958182896189[217] = 0.0;
   out_5583056958182896189[218] = 0.0;
   out_5583056958182896189[219] = 0.0;
   out_5583056958182896189[220] = 0.0;
   out_5583056958182896189[221] = 0.0;
   out_5583056958182896189[222] = 0.0;
   out_5583056958182896189[223] = 0.0;
   out_5583056958182896189[224] = 0.0;
   out_5583056958182896189[225] = 0.0;
   out_5583056958182896189[226] = 0.0;
   out_5583056958182896189[227] = 0.0;
   out_5583056958182896189[228] = 1.0;
   out_5583056958182896189[229] = 0.0;
   out_5583056958182896189[230] = 0.0;
   out_5583056958182896189[231] = 0.0;
   out_5583056958182896189[232] = 0.0;
   out_5583056958182896189[233] = 0.0;
   out_5583056958182896189[234] = 0.0;
   out_5583056958182896189[235] = 0.0;
   out_5583056958182896189[236] = 0.0;
   out_5583056958182896189[237] = 0.0;
   out_5583056958182896189[238] = 0.0;
   out_5583056958182896189[239] = 0.0;
   out_5583056958182896189[240] = 0.0;
   out_5583056958182896189[241] = 0.0;
   out_5583056958182896189[242] = 0.0;
   out_5583056958182896189[243] = 0.0;
   out_5583056958182896189[244] = 0.0;
   out_5583056958182896189[245] = 0.0;
   out_5583056958182896189[246] = 0.0;
   out_5583056958182896189[247] = 1.0;
   out_5583056958182896189[248] = 0.0;
   out_5583056958182896189[249] = 0.0;
   out_5583056958182896189[250] = 0.0;
   out_5583056958182896189[251] = 0.0;
   out_5583056958182896189[252] = 0.0;
   out_5583056958182896189[253] = 0.0;
   out_5583056958182896189[254] = 0.0;
   out_5583056958182896189[255] = 0.0;
   out_5583056958182896189[256] = 0.0;
   out_5583056958182896189[257] = 0.0;
   out_5583056958182896189[258] = 0.0;
   out_5583056958182896189[259] = 0.0;
   out_5583056958182896189[260] = 0.0;
   out_5583056958182896189[261] = 0.0;
   out_5583056958182896189[262] = 0.0;
   out_5583056958182896189[263] = 0.0;
   out_5583056958182896189[264] = 0.0;
   out_5583056958182896189[265] = 0.0;
   out_5583056958182896189[266] = 1.0;
   out_5583056958182896189[267] = 0.0;
   out_5583056958182896189[268] = 0.0;
   out_5583056958182896189[269] = 0.0;
   out_5583056958182896189[270] = 0.0;
   out_5583056958182896189[271] = 0.0;
   out_5583056958182896189[272] = 0.0;
   out_5583056958182896189[273] = 0.0;
   out_5583056958182896189[274] = 0.0;
   out_5583056958182896189[275] = 0.0;
   out_5583056958182896189[276] = 0.0;
   out_5583056958182896189[277] = 0.0;
   out_5583056958182896189[278] = 0.0;
   out_5583056958182896189[279] = 0.0;
   out_5583056958182896189[280] = 0.0;
   out_5583056958182896189[281] = 0.0;
   out_5583056958182896189[282] = 0.0;
   out_5583056958182896189[283] = 0.0;
   out_5583056958182896189[284] = 0.0;
   out_5583056958182896189[285] = 1.0;
   out_5583056958182896189[286] = 0.0;
   out_5583056958182896189[287] = 0.0;
   out_5583056958182896189[288] = 0.0;
   out_5583056958182896189[289] = 0.0;
   out_5583056958182896189[290] = 0.0;
   out_5583056958182896189[291] = 0.0;
   out_5583056958182896189[292] = 0.0;
   out_5583056958182896189[293] = 0.0;
   out_5583056958182896189[294] = 0.0;
   out_5583056958182896189[295] = 0.0;
   out_5583056958182896189[296] = 0.0;
   out_5583056958182896189[297] = 0.0;
   out_5583056958182896189[298] = 0.0;
   out_5583056958182896189[299] = 0.0;
   out_5583056958182896189[300] = 0.0;
   out_5583056958182896189[301] = 0.0;
   out_5583056958182896189[302] = 0.0;
   out_5583056958182896189[303] = 0.0;
   out_5583056958182896189[304] = 1.0;
   out_5583056958182896189[305] = 0.0;
   out_5583056958182896189[306] = 0.0;
   out_5583056958182896189[307] = 0.0;
   out_5583056958182896189[308] = 0.0;
   out_5583056958182896189[309] = 0.0;
   out_5583056958182896189[310] = 0.0;
   out_5583056958182896189[311] = 0.0;
   out_5583056958182896189[312] = 0.0;
   out_5583056958182896189[313] = 0.0;
   out_5583056958182896189[314] = 0.0;
   out_5583056958182896189[315] = 0.0;
   out_5583056958182896189[316] = 0.0;
   out_5583056958182896189[317] = 0.0;
   out_5583056958182896189[318] = 0.0;
   out_5583056958182896189[319] = 0.0;
   out_5583056958182896189[320] = 0.0;
   out_5583056958182896189[321] = 0.0;
   out_5583056958182896189[322] = 0.0;
   out_5583056958182896189[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6878832699624430321) {
   out_6878832699624430321[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6878832699624430321[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6878832699624430321[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6878832699624430321[3] = dt*state[12] + state[3];
   out_6878832699624430321[4] = dt*state[13] + state[4];
   out_6878832699624430321[5] = dt*state[14] + state[5];
   out_6878832699624430321[6] = state[6];
   out_6878832699624430321[7] = state[7];
   out_6878832699624430321[8] = state[8];
   out_6878832699624430321[9] = state[9];
   out_6878832699624430321[10] = state[10];
   out_6878832699624430321[11] = state[11];
   out_6878832699624430321[12] = state[12];
   out_6878832699624430321[13] = state[13];
   out_6878832699624430321[14] = state[14];
   out_6878832699624430321[15] = state[15];
   out_6878832699624430321[16] = state[16];
   out_6878832699624430321[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2457315738183324621) {
   out_2457315738183324621[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2457315738183324621[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2457315738183324621[2] = 0;
   out_2457315738183324621[3] = 0;
   out_2457315738183324621[4] = 0;
   out_2457315738183324621[5] = 0;
   out_2457315738183324621[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2457315738183324621[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2457315738183324621[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2457315738183324621[9] = 0;
   out_2457315738183324621[10] = 0;
   out_2457315738183324621[11] = 0;
   out_2457315738183324621[12] = 0;
   out_2457315738183324621[13] = 0;
   out_2457315738183324621[14] = 0;
   out_2457315738183324621[15] = 0;
   out_2457315738183324621[16] = 0;
   out_2457315738183324621[17] = 0;
   out_2457315738183324621[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2457315738183324621[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2457315738183324621[20] = 0;
   out_2457315738183324621[21] = 0;
   out_2457315738183324621[22] = 0;
   out_2457315738183324621[23] = 0;
   out_2457315738183324621[24] = 0;
   out_2457315738183324621[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2457315738183324621[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2457315738183324621[27] = 0;
   out_2457315738183324621[28] = 0;
   out_2457315738183324621[29] = 0;
   out_2457315738183324621[30] = 0;
   out_2457315738183324621[31] = 0;
   out_2457315738183324621[32] = 0;
   out_2457315738183324621[33] = 0;
   out_2457315738183324621[34] = 0;
   out_2457315738183324621[35] = 0;
   out_2457315738183324621[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2457315738183324621[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2457315738183324621[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2457315738183324621[39] = 0;
   out_2457315738183324621[40] = 0;
   out_2457315738183324621[41] = 0;
   out_2457315738183324621[42] = 0;
   out_2457315738183324621[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2457315738183324621[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2457315738183324621[45] = 0;
   out_2457315738183324621[46] = 0;
   out_2457315738183324621[47] = 0;
   out_2457315738183324621[48] = 0;
   out_2457315738183324621[49] = 0;
   out_2457315738183324621[50] = 0;
   out_2457315738183324621[51] = 0;
   out_2457315738183324621[52] = 0;
   out_2457315738183324621[53] = 0;
   out_2457315738183324621[54] = 0;
   out_2457315738183324621[55] = 0;
   out_2457315738183324621[56] = 0;
   out_2457315738183324621[57] = 1;
   out_2457315738183324621[58] = 0;
   out_2457315738183324621[59] = 0;
   out_2457315738183324621[60] = 0;
   out_2457315738183324621[61] = 0;
   out_2457315738183324621[62] = 0;
   out_2457315738183324621[63] = 0;
   out_2457315738183324621[64] = 0;
   out_2457315738183324621[65] = 0;
   out_2457315738183324621[66] = dt;
   out_2457315738183324621[67] = 0;
   out_2457315738183324621[68] = 0;
   out_2457315738183324621[69] = 0;
   out_2457315738183324621[70] = 0;
   out_2457315738183324621[71] = 0;
   out_2457315738183324621[72] = 0;
   out_2457315738183324621[73] = 0;
   out_2457315738183324621[74] = 0;
   out_2457315738183324621[75] = 0;
   out_2457315738183324621[76] = 1;
   out_2457315738183324621[77] = 0;
   out_2457315738183324621[78] = 0;
   out_2457315738183324621[79] = 0;
   out_2457315738183324621[80] = 0;
   out_2457315738183324621[81] = 0;
   out_2457315738183324621[82] = 0;
   out_2457315738183324621[83] = 0;
   out_2457315738183324621[84] = 0;
   out_2457315738183324621[85] = dt;
   out_2457315738183324621[86] = 0;
   out_2457315738183324621[87] = 0;
   out_2457315738183324621[88] = 0;
   out_2457315738183324621[89] = 0;
   out_2457315738183324621[90] = 0;
   out_2457315738183324621[91] = 0;
   out_2457315738183324621[92] = 0;
   out_2457315738183324621[93] = 0;
   out_2457315738183324621[94] = 0;
   out_2457315738183324621[95] = 1;
   out_2457315738183324621[96] = 0;
   out_2457315738183324621[97] = 0;
   out_2457315738183324621[98] = 0;
   out_2457315738183324621[99] = 0;
   out_2457315738183324621[100] = 0;
   out_2457315738183324621[101] = 0;
   out_2457315738183324621[102] = 0;
   out_2457315738183324621[103] = 0;
   out_2457315738183324621[104] = dt;
   out_2457315738183324621[105] = 0;
   out_2457315738183324621[106] = 0;
   out_2457315738183324621[107] = 0;
   out_2457315738183324621[108] = 0;
   out_2457315738183324621[109] = 0;
   out_2457315738183324621[110] = 0;
   out_2457315738183324621[111] = 0;
   out_2457315738183324621[112] = 0;
   out_2457315738183324621[113] = 0;
   out_2457315738183324621[114] = 1;
   out_2457315738183324621[115] = 0;
   out_2457315738183324621[116] = 0;
   out_2457315738183324621[117] = 0;
   out_2457315738183324621[118] = 0;
   out_2457315738183324621[119] = 0;
   out_2457315738183324621[120] = 0;
   out_2457315738183324621[121] = 0;
   out_2457315738183324621[122] = 0;
   out_2457315738183324621[123] = 0;
   out_2457315738183324621[124] = 0;
   out_2457315738183324621[125] = 0;
   out_2457315738183324621[126] = 0;
   out_2457315738183324621[127] = 0;
   out_2457315738183324621[128] = 0;
   out_2457315738183324621[129] = 0;
   out_2457315738183324621[130] = 0;
   out_2457315738183324621[131] = 0;
   out_2457315738183324621[132] = 0;
   out_2457315738183324621[133] = 1;
   out_2457315738183324621[134] = 0;
   out_2457315738183324621[135] = 0;
   out_2457315738183324621[136] = 0;
   out_2457315738183324621[137] = 0;
   out_2457315738183324621[138] = 0;
   out_2457315738183324621[139] = 0;
   out_2457315738183324621[140] = 0;
   out_2457315738183324621[141] = 0;
   out_2457315738183324621[142] = 0;
   out_2457315738183324621[143] = 0;
   out_2457315738183324621[144] = 0;
   out_2457315738183324621[145] = 0;
   out_2457315738183324621[146] = 0;
   out_2457315738183324621[147] = 0;
   out_2457315738183324621[148] = 0;
   out_2457315738183324621[149] = 0;
   out_2457315738183324621[150] = 0;
   out_2457315738183324621[151] = 0;
   out_2457315738183324621[152] = 1;
   out_2457315738183324621[153] = 0;
   out_2457315738183324621[154] = 0;
   out_2457315738183324621[155] = 0;
   out_2457315738183324621[156] = 0;
   out_2457315738183324621[157] = 0;
   out_2457315738183324621[158] = 0;
   out_2457315738183324621[159] = 0;
   out_2457315738183324621[160] = 0;
   out_2457315738183324621[161] = 0;
   out_2457315738183324621[162] = 0;
   out_2457315738183324621[163] = 0;
   out_2457315738183324621[164] = 0;
   out_2457315738183324621[165] = 0;
   out_2457315738183324621[166] = 0;
   out_2457315738183324621[167] = 0;
   out_2457315738183324621[168] = 0;
   out_2457315738183324621[169] = 0;
   out_2457315738183324621[170] = 0;
   out_2457315738183324621[171] = 1;
   out_2457315738183324621[172] = 0;
   out_2457315738183324621[173] = 0;
   out_2457315738183324621[174] = 0;
   out_2457315738183324621[175] = 0;
   out_2457315738183324621[176] = 0;
   out_2457315738183324621[177] = 0;
   out_2457315738183324621[178] = 0;
   out_2457315738183324621[179] = 0;
   out_2457315738183324621[180] = 0;
   out_2457315738183324621[181] = 0;
   out_2457315738183324621[182] = 0;
   out_2457315738183324621[183] = 0;
   out_2457315738183324621[184] = 0;
   out_2457315738183324621[185] = 0;
   out_2457315738183324621[186] = 0;
   out_2457315738183324621[187] = 0;
   out_2457315738183324621[188] = 0;
   out_2457315738183324621[189] = 0;
   out_2457315738183324621[190] = 1;
   out_2457315738183324621[191] = 0;
   out_2457315738183324621[192] = 0;
   out_2457315738183324621[193] = 0;
   out_2457315738183324621[194] = 0;
   out_2457315738183324621[195] = 0;
   out_2457315738183324621[196] = 0;
   out_2457315738183324621[197] = 0;
   out_2457315738183324621[198] = 0;
   out_2457315738183324621[199] = 0;
   out_2457315738183324621[200] = 0;
   out_2457315738183324621[201] = 0;
   out_2457315738183324621[202] = 0;
   out_2457315738183324621[203] = 0;
   out_2457315738183324621[204] = 0;
   out_2457315738183324621[205] = 0;
   out_2457315738183324621[206] = 0;
   out_2457315738183324621[207] = 0;
   out_2457315738183324621[208] = 0;
   out_2457315738183324621[209] = 1;
   out_2457315738183324621[210] = 0;
   out_2457315738183324621[211] = 0;
   out_2457315738183324621[212] = 0;
   out_2457315738183324621[213] = 0;
   out_2457315738183324621[214] = 0;
   out_2457315738183324621[215] = 0;
   out_2457315738183324621[216] = 0;
   out_2457315738183324621[217] = 0;
   out_2457315738183324621[218] = 0;
   out_2457315738183324621[219] = 0;
   out_2457315738183324621[220] = 0;
   out_2457315738183324621[221] = 0;
   out_2457315738183324621[222] = 0;
   out_2457315738183324621[223] = 0;
   out_2457315738183324621[224] = 0;
   out_2457315738183324621[225] = 0;
   out_2457315738183324621[226] = 0;
   out_2457315738183324621[227] = 0;
   out_2457315738183324621[228] = 1;
   out_2457315738183324621[229] = 0;
   out_2457315738183324621[230] = 0;
   out_2457315738183324621[231] = 0;
   out_2457315738183324621[232] = 0;
   out_2457315738183324621[233] = 0;
   out_2457315738183324621[234] = 0;
   out_2457315738183324621[235] = 0;
   out_2457315738183324621[236] = 0;
   out_2457315738183324621[237] = 0;
   out_2457315738183324621[238] = 0;
   out_2457315738183324621[239] = 0;
   out_2457315738183324621[240] = 0;
   out_2457315738183324621[241] = 0;
   out_2457315738183324621[242] = 0;
   out_2457315738183324621[243] = 0;
   out_2457315738183324621[244] = 0;
   out_2457315738183324621[245] = 0;
   out_2457315738183324621[246] = 0;
   out_2457315738183324621[247] = 1;
   out_2457315738183324621[248] = 0;
   out_2457315738183324621[249] = 0;
   out_2457315738183324621[250] = 0;
   out_2457315738183324621[251] = 0;
   out_2457315738183324621[252] = 0;
   out_2457315738183324621[253] = 0;
   out_2457315738183324621[254] = 0;
   out_2457315738183324621[255] = 0;
   out_2457315738183324621[256] = 0;
   out_2457315738183324621[257] = 0;
   out_2457315738183324621[258] = 0;
   out_2457315738183324621[259] = 0;
   out_2457315738183324621[260] = 0;
   out_2457315738183324621[261] = 0;
   out_2457315738183324621[262] = 0;
   out_2457315738183324621[263] = 0;
   out_2457315738183324621[264] = 0;
   out_2457315738183324621[265] = 0;
   out_2457315738183324621[266] = 1;
   out_2457315738183324621[267] = 0;
   out_2457315738183324621[268] = 0;
   out_2457315738183324621[269] = 0;
   out_2457315738183324621[270] = 0;
   out_2457315738183324621[271] = 0;
   out_2457315738183324621[272] = 0;
   out_2457315738183324621[273] = 0;
   out_2457315738183324621[274] = 0;
   out_2457315738183324621[275] = 0;
   out_2457315738183324621[276] = 0;
   out_2457315738183324621[277] = 0;
   out_2457315738183324621[278] = 0;
   out_2457315738183324621[279] = 0;
   out_2457315738183324621[280] = 0;
   out_2457315738183324621[281] = 0;
   out_2457315738183324621[282] = 0;
   out_2457315738183324621[283] = 0;
   out_2457315738183324621[284] = 0;
   out_2457315738183324621[285] = 1;
   out_2457315738183324621[286] = 0;
   out_2457315738183324621[287] = 0;
   out_2457315738183324621[288] = 0;
   out_2457315738183324621[289] = 0;
   out_2457315738183324621[290] = 0;
   out_2457315738183324621[291] = 0;
   out_2457315738183324621[292] = 0;
   out_2457315738183324621[293] = 0;
   out_2457315738183324621[294] = 0;
   out_2457315738183324621[295] = 0;
   out_2457315738183324621[296] = 0;
   out_2457315738183324621[297] = 0;
   out_2457315738183324621[298] = 0;
   out_2457315738183324621[299] = 0;
   out_2457315738183324621[300] = 0;
   out_2457315738183324621[301] = 0;
   out_2457315738183324621[302] = 0;
   out_2457315738183324621[303] = 0;
   out_2457315738183324621[304] = 1;
   out_2457315738183324621[305] = 0;
   out_2457315738183324621[306] = 0;
   out_2457315738183324621[307] = 0;
   out_2457315738183324621[308] = 0;
   out_2457315738183324621[309] = 0;
   out_2457315738183324621[310] = 0;
   out_2457315738183324621[311] = 0;
   out_2457315738183324621[312] = 0;
   out_2457315738183324621[313] = 0;
   out_2457315738183324621[314] = 0;
   out_2457315738183324621[315] = 0;
   out_2457315738183324621[316] = 0;
   out_2457315738183324621[317] = 0;
   out_2457315738183324621[318] = 0;
   out_2457315738183324621[319] = 0;
   out_2457315738183324621[320] = 0;
   out_2457315738183324621[321] = 0;
   out_2457315738183324621[322] = 0;
   out_2457315738183324621[323] = 1;
}
void h_4(double *state, double *unused, double *out_7759867018497998577) {
   out_7759867018497998577[0] = state[6] + state[9];
   out_7759867018497998577[1] = state[7] + state[10];
   out_7759867018497998577[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3361304707874925153) {
   out_3361304707874925153[0] = 0;
   out_3361304707874925153[1] = 0;
   out_3361304707874925153[2] = 0;
   out_3361304707874925153[3] = 0;
   out_3361304707874925153[4] = 0;
   out_3361304707874925153[5] = 0;
   out_3361304707874925153[6] = 1;
   out_3361304707874925153[7] = 0;
   out_3361304707874925153[8] = 0;
   out_3361304707874925153[9] = 1;
   out_3361304707874925153[10] = 0;
   out_3361304707874925153[11] = 0;
   out_3361304707874925153[12] = 0;
   out_3361304707874925153[13] = 0;
   out_3361304707874925153[14] = 0;
   out_3361304707874925153[15] = 0;
   out_3361304707874925153[16] = 0;
   out_3361304707874925153[17] = 0;
   out_3361304707874925153[18] = 0;
   out_3361304707874925153[19] = 0;
   out_3361304707874925153[20] = 0;
   out_3361304707874925153[21] = 0;
   out_3361304707874925153[22] = 0;
   out_3361304707874925153[23] = 0;
   out_3361304707874925153[24] = 0;
   out_3361304707874925153[25] = 1;
   out_3361304707874925153[26] = 0;
   out_3361304707874925153[27] = 0;
   out_3361304707874925153[28] = 1;
   out_3361304707874925153[29] = 0;
   out_3361304707874925153[30] = 0;
   out_3361304707874925153[31] = 0;
   out_3361304707874925153[32] = 0;
   out_3361304707874925153[33] = 0;
   out_3361304707874925153[34] = 0;
   out_3361304707874925153[35] = 0;
   out_3361304707874925153[36] = 0;
   out_3361304707874925153[37] = 0;
   out_3361304707874925153[38] = 0;
   out_3361304707874925153[39] = 0;
   out_3361304707874925153[40] = 0;
   out_3361304707874925153[41] = 0;
   out_3361304707874925153[42] = 0;
   out_3361304707874925153[43] = 0;
   out_3361304707874925153[44] = 1;
   out_3361304707874925153[45] = 0;
   out_3361304707874925153[46] = 0;
   out_3361304707874925153[47] = 1;
   out_3361304707874925153[48] = 0;
   out_3361304707874925153[49] = 0;
   out_3361304707874925153[50] = 0;
   out_3361304707874925153[51] = 0;
   out_3361304707874925153[52] = 0;
   out_3361304707874925153[53] = 0;
}
void h_10(double *state, double *unused, double *out_5255984674989194447) {
   out_5255984674989194447[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5255984674989194447[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5255984674989194447[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_9077457845341768252) {
   out_9077457845341768252[0] = 0;
   out_9077457845341768252[1] = 9.8100000000000005*cos(state[1]);
   out_9077457845341768252[2] = 0;
   out_9077457845341768252[3] = 0;
   out_9077457845341768252[4] = -state[8];
   out_9077457845341768252[5] = state[7];
   out_9077457845341768252[6] = 0;
   out_9077457845341768252[7] = state[5];
   out_9077457845341768252[8] = -state[4];
   out_9077457845341768252[9] = 0;
   out_9077457845341768252[10] = 0;
   out_9077457845341768252[11] = 0;
   out_9077457845341768252[12] = 1;
   out_9077457845341768252[13] = 0;
   out_9077457845341768252[14] = 0;
   out_9077457845341768252[15] = 1;
   out_9077457845341768252[16] = 0;
   out_9077457845341768252[17] = 0;
   out_9077457845341768252[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_9077457845341768252[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_9077457845341768252[20] = 0;
   out_9077457845341768252[21] = state[8];
   out_9077457845341768252[22] = 0;
   out_9077457845341768252[23] = -state[6];
   out_9077457845341768252[24] = -state[5];
   out_9077457845341768252[25] = 0;
   out_9077457845341768252[26] = state[3];
   out_9077457845341768252[27] = 0;
   out_9077457845341768252[28] = 0;
   out_9077457845341768252[29] = 0;
   out_9077457845341768252[30] = 0;
   out_9077457845341768252[31] = 1;
   out_9077457845341768252[32] = 0;
   out_9077457845341768252[33] = 0;
   out_9077457845341768252[34] = 1;
   out_9077457845341768252[35] = 0;
   out_9077457845341768252[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_9077457845341768252[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_9077457845341768252[38] = 0;
   out_9077457845341768252[39] = -state[7];
   out_9077457845341768252[40] = state[6];
   out_9077457845341768252[41] = 0;
   out_9077457845341768252[42] = state[4];
   out_9077457845341768252[43] = -state[3];
   out_9077457845341768252[44] = 0;
   out_9077457845341768252[45] = 0;
   out_9077457845341768252[46] = 0;
   out_9077457845341768252[47] = 0;
   out_9077457845341768252[48] = 0;
   out_9077457845341768252[49] = 0;
   out_9077457845341768252[50] = 1;
   out_9077457845341768252[51] = 0;
   out_9077457845341768252[52] = 0;
   out_9077457845341768252[53] = 1;
}
void h_13(double *state, double *unused, double *out_5774649359282809780) {
   out_5774649359282809780[0] = state[3];
   out_5774649359282809780[1] = state[4];
   out_5774649359282809780[2] = state[5];
}
void H_13(double *state, double *unused, double *out_149030882542592352) {
   out_149030882542592352[0] = 0;
   out_149030882542592352[1] = 0;
   out_149030882542592352[2] = 0;
   out_149030882542592352[3] = 1;
   out_149030882542592352[4] = 0;
   out_149030882542592352[5] = 0;
   out_149030882542592352[6] = 0;
   out_149030882542592352[7] = 0;
   out_149030882542592352[8] = 0;
   out_149030882542592352[9] = 0;
   out_149030882542592352[10] = 0;
   out_149030882542592352[11] = 0;
   out_149030882542592352[12] = 0;
   out_149030882542592352[13] = 0;
   out_149030882542592352[14] = 0;
   out_149030882542592352[15] = 0;
   out_149030882542592352[16] = 0;
   out_149030882542592352[17] = 0;
   out_149030882542592352[18] = 0;
   out_149030882542592352[19] = 0;
   out_149030882542592352[20] = 0;
   out_149030882542592352[21] = 0;
   out_149030882542592352[22] = 1;
   out_149030882542592352[23] = 0;
   out_149030882542592352[24] = 0;
   out_149030882542592352[25] = 0;
   out_149030882542592352[26] = 0;
   out_149030882542592352[27] = 0;
   out_149030882542592352[28] = 0;
   out_149030882542592352[29] = 0;
   out_149030882542592352[30] = 0;
   out_149030882542592352[31] = 0;
   out_149030882542592352[32] = 0;
   out_149030882542592352[33] = 0;
   out_149030882542592352[34] = 0;
   out_149030882542592352[35] = 0;
   out_149030882542592352[36] = 0;
   out_149030882542592352[37] = 0;
   out_149030882542592352[38] = 0;
   out_149030882542592352[39] = 0;
   out_149030882542592352[40] = 0;
   out_149030882542592352[41] = 1;
   out_149030882542592352[42] = 0;
   out_149030882542592352[43] = 0;
   out_149030882542592352[44] = 0;
   out_149030882542592352[45] = 0;
   out_149030882542592352[46] = 0;
   out_149030882542592352[47] = 0;
   out_149030882542592352[48] = 0;
   out_149030882542592352[49] = 0;
   out_149030882542592352[50] = 0;
   out_149030882542592352[51] = 0;
   out_149030882542592352[52] = 0;
   out_149030882542592352[53] = 0;
}
void h_14(double *state, double *unused, double *out_5470742825461452978) {
   out_5470742825461452978[0] = state[6];
   out_5470742825461452978[1] = state[7];
   out_5470742825461452978[2] = state[8];
}
void H_14(double *state, double *unused, double *out_601936148464559376) {
   out_601936148464559376[0] = 0;
   out_601936148464559376[1] = 0;
   out_601936148464559376[2] = 0;
   out_601936148464559376[3] = 0;
   out_601936148464559376[4] = 0;
   out_601936148464559376[5] = 0;
   out_601936148464559376[6] = 1;
   out_601936148464559376[7] = 0;
   out_601936148464559376[8] = 0;
   out_601936148464559376[9] = 0;
   out_601936148464559376[10] = 0;
   out_601936148464559376[11] = 0;
   out_601936148464559376[12] = 0;
   out_601936148464559376[13] = 0;
   out_601936148464559376[14] = 0;
   out_601936148464559376[15] = 0;
   out_601936148464559376[16] = 0;
   out_601936148464559376[17] = 0;
   out_601936148464559376[18] = 0;
   out_601936148464559376[19] = 0;
   out_601936148464559376[20] = 0;
   out_601936148464559376[21] = 0;
   out_601936148464559376[22] = 0;
   out_601936148464559376[23] = 0;
   out_601936148464559376[24] = 0;
   out_601936148464559376[25] = 1;
   out_601936148464559376[26] = 0;
   out_601936148464559376[27] = 0;
   out_601936148464559376[28] = 0;
   out_601936148464559376[29] = 0;
   out_601936148464559376[30] = 0;
   out_601936148464559376[31] = 0;
   out_601936148464559376[32] = 0;
   out_601936148464559376[33] = 0;
   out_601936148464559376[34] = 0;
   out_601936148464559376[35] = 0;
   out_601936148464559376[36] = 0;
   out_601936148464559376[37] = 0;
   out_601936148464559376[38] = 0;
   out_601936148464559376[39] = 0;
   out_601936148464559376[40] = 0;
   out_601936148464559376[41] = 0;
   out_601936148464559376[42] = 0;
   out_601936148464559376[43] = 0;
   out_601936148464559376[44] = 1;
   out_601936148464559376[45] = 0;
   out_601936148464559376[46] = 0;
   out_601936148464559376[47] = 0;
   out_601936148464559376[48] = 0;
   out_601936148464559376[49] = 0;
   out_601936148464559376[50] = 0;
   out_601936148464559376[51] = 0;
   out_601936148464559376[52] = 0;
   out_601936148464559376[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_1886515091563246719) {
  err_fun(nom_x, delta_x, out_1886515091563246719);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8728219793499801644) {
  inv_err_fun(nom_x, true_x, out_8728219793499801644);
}
void pose_H_mod_fun(double *state, double *out_5583056958182896189) {
  H_mod_fun(state, out_5583056958182896189);
}
void pose_f_fun(double *state, double dt, double *out_6878832699624430321) {
  f_fun(state,  dt, out_6878832699624430321);
}
void pose_F_fun(double *state, double dt, double *out_2457315738183324621) {
  F_fun(state,  dt, out_2457315738183324621);
}
void pose_h_4(double *state, double *unused, double *out_7759867018497998577) {
  h_4(state, unused, out_7759867018497998577);
}
void pose_H_4(double *state, double *unused, double *out_3361304707874925153) {
  H_4(state, unused, out_3361304707874925153);
}
void pose_h_10(double *state, double *unused, double *out_5255984674989194447) {
  h_10(state, unused, out_5255984674989194447);
}
void pose_H_10(double *state, double *unused, double *out_9077457845341768252) {
  H_10(state, unused, out_9077457845341768252);
}
void pose_h_13(double *state, double *unused, double *out_5774649359282809780) {
  h_13(state, unused, out_5774649359282809780);
}
void pose_H_13(double *state, double *unused, double *out_149030882542592352) {
  H_13(state, unused, out_149030882542592352);
}
void pose_h_14(double *state, double *unused, double *out_5470742825461452978) {
  h_14(state, unused, out_5470742825461452978);
}
void pose_H_14(double *state, double *unused, double *out_601936148464559376) {
  H_14(state, unused, out_601936148464559376);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
