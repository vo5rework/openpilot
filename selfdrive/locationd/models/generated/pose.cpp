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
void err_fun(double *nom_x, double *delta_x, double *out_2768303435411887346) {
   out_2768303435411887346[0] = delta_x[0] + nom_x[0];
   out_2768303435411887346[1] = delta_x[1] + nom_x[1];
   out_2768303435411887346[2] = delta_x[2] + nom_x[2];
   out_2768303435411887346[3] = delta_x[3] + nom_x[3];
   out_2768303435411887346[4] = delta_x[4] + nom_x[4];
   out_2768303435411887346[5] = delta_x[5] + nom_x[5];
   out_2768303435411887346[6] = delta_x[6] + nom_x[6];
   out_2768303435411887346[7] = delta_x[7] + nom_x[7];
   out_2768303435411887346[8] = delta_x[8] + nom_x[8];
   out_2768303435411887346[9] = delta_x[9] + nom_x[9];
   out_2768303435411887346[10] = delta_x[10] + nom_x[10];
   out_2768303435411887346[11] = delta_x[11] + nom_x[11];
   out_2768303435411887346[12] = delta_x[12] + nom_x[12];
   out_2768303435411887346[13] = delta_x[13] + nom_x[13];
   out_2768303435411887346[14] = delta_x[14] + nom_x[14];
   out_2768303435411887346[15] = delta_x[15] + nom_x[15];
   out_2768303435411887346[16] = delta_x[16] + nom_x[16];
   out_2768303435411887346[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1488408519072955526) {
   out_1488408519072955526[0] = -nom_x[0] + true_x[0];
   out_1488408519072955526[1] = -nom_x[1] + true_x[1];
   out_1488408519072955526[2] = -nom_x[2] + true_x[2];
   out_1488408519072955526[3] = -nom_x[3] + true_x[3];
   out_1488408519072955526[4] = -nom_x[4] + true_x[4];
   out_1488408519072955526[5] = -nom_x[5] + true_x[5];
   out_1488408519072955526[6] = -nom_x[6] + true_x[6];
   out_1488408519072955526[7] = -nom_x[7] + true_x[7];
   out_1488408519072955526[8] = -nom_x[8] + true_x[8];
   out_1488408519072955526[9] = -nom_x[9] + true_x[9];
   out_1488408519072955526[10] = -nom_x[10] + true_x[10];
   out_1488408519072955526[11] = -nom_x[11] + true_x[11];
   out_1488408519072955526[12] = -nom_x[12] + true_x[12];
   out_1488408519072955526[13] = -nom_x[13] + true_x[13];
   out_1488408519072955526[14] = -nom_x[14] + true_x[14];
   out_1488408519072955526[15] = -nom_x[15] + true_x[15];
   out_1488408519072955526[16] = -nom_x[16] + true_x[16];
   out_1488408519072955526[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1747102118856192496) {
   out_1747102118856192496[0] = 1.0;
   out_1747102118856192496[1] = 0.0;
   out_1747102118856192496[2] = 0.0;
   out_1747102118856192496[3] = 0.0;
   out_1747102118856192496[4] = 0.0;
   out_1747102118856192496[5] = 0.0;
   out_1747102118856192496[6] = 0.0;
   out_1747102118856192496[7] = 0.0;
   out_1747102118856192496[8] = 0.0;
   out_1747102118856192496[9] = 0.0;
   out_1747102118856192496[10] = 0.0;
   out_1747102118856192496[11] = 0.0;
   out_1747102118856192496[12] = 0.0;
   out_1747102118856192496[13] = 0.0;
   out_1747102118856192496[14] = 0.0;
   out_1747102118856192496[15] = 0.0;
   out_1747102118856192496[16] = 0.0;
   out_1747102118856192496[17] = 0.0;
   out_1747102118856192496[18] = 0.0;
   out_1747102118856192496[19] = 1.0;
   out_1747102118856192496[20] = 0.0;
   out_1747102118856192496[21] = 0.0;
   out_1747102118856192496[22] = 0.0;
   out_1747102118856192496[23] = 0.0;
   out_1747102118856192496[24] = 0.0;
   out_1747102118856192496[25] = 0.0;
   out_1747102118856192496[26] = 0.0;
   out_1747102118856192496[27] = 0.0;
   out_1747102118856192496[28] = 0.0;
   out_1747102118856192496[29] = 0.0;
   out_1747102118856192496[30] = 0.0;
   out_1747102118856192496[31] = 0.0;
   out_1747102118856192496[32] = 0.0;
   out_1747102118856192496[33] = 0.0;
   out_1747102118856192496[34] = 0.0;
   out_1747102118856192496[35] = 0.0;
   out_1747102118856192496[36] = 0.0;
   out_1747102118856192496[37] = 0.0;
   out_1747102118856192496[38] = 1.0;
   out_1747102118856192496[39] = 0.0;
   out_1747102118856192496[40] = 0.0;
   out_1747102118856192496[41] = 0.0;
   out_1747102118856192496[42] = 0.0;
   out_1747102118856192496[43] = 0.0;
   out_1747102118856192496[44] = 0.0;
   out_1747102118856192496[45] = 0.0;
   out_1747102118856192496[46] = 0.0;
   out_1747102118856192496[47] = 0.0;
   out_1747102118856192496[48] = 0.0;
   out_1747102118856192496[49] = 0.0;
   out_1747102118856192496[50] = 0.0;
   out_1747102118856192496[51] = 0.0;
   out_1747102118856192496[52] = 0.0;
   out_1747102118856192496[53] = 0.0;
   out_1747102118856192496[54] = 0.0;
   out_1747102118856192496[55] = 0.0;
   out_1747102118856192496[56] = 0.0;
   out_1747102118856192496[57] = 1.0;
   out_1747102118856192496[58] = 0.0;
   out_1747102118856192496[59] = 0.0;
   out_1747102118856192496[60] = 0.0;
   out_1747102118856192496[61] = 0.0;
   out_1747102118856192496[62] = 0.0;
   out_1747102118856192496[63] = 0.0;
   out_1747102118856192496[64] = 0.0;
   out_1747102118856192496[65] = 0.0;
   out_1747102118856192496[66] = 0.0;
   out_1747102118856192496[67] = 0.0;
   out_1747102118856192496[68] = 0.0;
   out_1747102118856192496[69] = 0.0;
   out_1747102118856192496[70] = 0.0;
   out_1747102118856192496[71] = 0.0;
   out_1747102118856192496[72] = 0.0;
   out_1747102118856192496[73] = 0.0;
   out_1747102118856192496[74] = 0.0;
   out_1747102118856192496[75] = 0.0;
   out_1747102118856192496[76] = 1.0;
   out_1747102118856192496[77] = 0.0;
   out_1747102118856192496[78] = 0.0;
   out_1747102118856192496[79] = 0.0;
   out_1747102118856192496[80] = 0.0;
   out_1747102118856192496[81] = 0.0;
   out_1747102118856192496[82] = 0.0;
   out_1747102118856192496[83] = 0.0;
   out_1747102118856192496[84] = 0.0;
   out_1747102118856192496[85] = 0.0;
   out_1747102118856192496[86] = 0.0;
   out_1747102118856192496[87] = 0.0;
   out_1747102118856192496[88] = 0.0;
   out_1747102118856192496[89] = 0.0;
   out_1747102118856192496[90] = 0.0;
   out_1747102118856192496[91] = 0.0;
   out_1747102118856192496[92] = 0.0;
   out_1747102118856192496[93] = 0.0;
   out_1747102118856192496[94] = 0.0;
   out_1747102118856192496[95] = 1.0;
   out_1747102118856192496[96] = 0.0;
   out_1747102118856192496[97] = 0.0;
   out_1747102118856192496[98] = 0.0;
   out_1747102118856192496[99] = 0.0;
   out_1747102118856192496[100] = 0.0;
   out_1747102118856192496[101] = 0.0;
   out_1747102118856192496[102] = 0.0;
   out_1747102118856192496[103] = 0.0;
   out_1747102118856192496[104] = 0.0;
   out_1747102118856192496[105] = 0.0;
   out_1747102118856192496[106] = 0.0;
   out_1747102118856192496[107] = 0.0;
   out_1747102118856192496[108] = 0.0;
   out_1747102118856192496[109] = 0.0;
   out_1747102118856192496[110] = 0.0;
   out_1747102118856192496[111] = 0.0;
   out_1747102118856192496[112] = 0.0;
   out_1747102118856192496[113] = 0.0;
   out_1747102118856192496[114] = 1.0;
   out_1747102118856192496[115] = 0.0;
   out_1747102118856192496[116] = 0.0;
   out_1747102118856192496[117] = 0.0;
   out_1747102118856192496[118] = 0.0;
   out_1747102118856192496[119] = 0.0;
   out_1747102118856192496[120] = 0.0;
   out_1747102118856192496[121] = 0.0;
   out_1747102118856192496[122] = 0.0;
   out_1747102118856192496[123] = 0.0;
   out_1747102118856192496[124] = 0.0;
   out_1747102118856192496[125] = 0.0;
   out_1747102118856192496[126] = 0.0;
   out_1747102118856192496[127] = 0.0;
   out_1747102118856192496[128] = 0.0;
   out_1747102118856192496[129] = 0.0;
   out_1747102118856192496[130] = 0.0;
   out_1747102118856192496[131] = 0.0;
   out_1747102118856192496[132] = 0.0;
   out_1747102118856192496[133] = 1.0;
   out_1747102118856192496[134] = 0.0;
   out_1747102118856192496[135] = 0.0;
   out_1747102118856192496[136] = 0.0;
   out_1747102118856192496[137] = 0.0;
   out_1747102118856192496[138] = 0.0;
   out_1747102118856192496[139] = 0.0;
   out_1747102118856192496[140] = 0.0;
   out_1747102118856192496[141] = 0.0;
   out_1747102118856192496[142] = 0.0;
   out_1747102118856192496[143] = 0.0;
   out_1747102118856192496[144] = 0.0;
   out_1747102118856192496[145] = 0.0;
   out_1747102118856192496[146] = 0.0;
   out_1747102118856192496[147] = 0.0;
   out_1747102118856192496[148] = 0.0;
   out_1747102118856192496[149] = 0.0;
   out_1747102118856192496[150] = 0.0;
   out_1747102118856192496[151] = 0.0;
   out_1747102118856192496[152] = 1.0;
   out_1747102118856192496[153] = 0.0;
   out_1747102118856192496[154] = 0.0;
   out_1747102118856192496[155] = 0.0;
   out_1747102118856192496[156] = 0.0;
   out_1747102118856192496[157] = 0.0;
   out_1747102118856192496[158] = 0.0;
   out_1747102118856192496[159] = 0.0;
   out_1747102118856192496[160] = 0.0;
   out_1747102118856192496[161] = 0.0;
   out_1747102118856192496[162] = 0.0;
   out_1747102118856192496[163] = 0.0;
   out_1747102118856192496[164] = 0.0;
   out_1747102118856192496[165] = 0.0;
   out_1747102118856192496[166] = 0.0;
   out_1747102118856192496[167] = 0.0;
   out_1747102118856192496[168] = 0.0;
   out_1747102118856192496[169] = 0.0;
   out_1747102118856192496[170] = 0.0;
   out_1747102118856192496[171] = 1.0;
   out_1747102118856192496[172] = 0.0;
   out_1747102118856192496[173] = 0.0;
   out_1747102118856192496[174] = 0.0;
   out_1747102118856192496[175] = 0.0;
   out_1747102118856192496[176] = 0.0;
   out_1747102118856192496[177] = 0.0;
   out_1747102118856192496[178] = 0.0;
   out_1747102118856192496[179] = 0.0;
   out_1747102118856192496[180] = 0.0;
   out_1747102118856192496[181] = 0.0;
   out_1747102118856192496[182] = 0.0;
   out_1747102118856192496[183] = 0.0;
   out_1747102118856192496[184] = 0.0;
   out_1747102118856192496[185] = 0.0;
   out_1747102118856192496[186] = 0.0;
   out_1747102118856192496[187] = 0.0;
   out_1747102118856192496[188] = 0.0;
   out_1747102118856192496[189] = 0.0;
   out_1747102118856192496[190] = 1.0;
   out_1747102118856192496[191] = 0.0;
   out_1747102118856192496[192] = 0.0;
   out_1747102118856192496[193] = 0.0;
   out_1747102118856192496[194] = 0.0;
   out_1747102118856192496[195] = 0.0;
   out_1747102118856192496[196] = 0.0;
   out_1747102118856192496[197] = 0.0;
   out_1747102118856192496[198] = 0.0;
   out_1747102118856192496[199] = 0.0;
   out_1747102118856192496[200] = 0.0;
   out_1747102118856192496[201] = 0.0;
   out_1747102118856192496[202] = 0.0;
   out_1747102118856192496[203] = 0.0;
   out_1747102118856192496[204] = 0.0;
   out_1747102118856192496[205] = 0.0;
   out_1747102118856192496[206] = 0.0;
   out_1747102118856192496[207] = 0.0;
   out_1747102118856192496[208] = 0.0;
   out_1747102118856192496[209] = 1.0;
   out_1747102118856192496[210] = 0.0;
   out_1747102118856192496[211] = 0.0;
   out_1747102118856192496[212] = 0.0;
   out_1747102118856192496[213] = 0.0;
   out_1747102118856192496[214] = 0.0;
   out_1747102118856192496[215] = 0.0;
   out_1747102118856192496[216] = 0.0;
   out_1747102118856192496[217] = 0.0;
   out_1747102118856192496[218] = 0.0;
   out_1747102118856192496[219] = 0.0;
   out_1747102118856192496[220] = 0.0;
   out_1747102118856192496[221] = 0.0;
   out_1747102118856192496[222] = 0.0;
   out_1747102118856192496[223] = 0.0;
   out_1747102118856192496[224] = 0.0;
   out_1747102118856192496[225] = 0.0;
   out_1747102118856192496[226] = 0.0;
   out_1747102118856192496[227] = 0.0;
   out_1747102118856192496[228] = 1.0;
   out_1747102118856192496[229] = 0.0;
   out_1747102118856192496[230] = 0.0;
   out_1747102118856192496[231] = 0.0;
   out_1747102118856192496[232] = 0.0;
   out_1747102118856192496[233] = 0.0;
   out_1747102118856192496[234] = 0.0;
   out_1747102118856192496[235] = 0.0;
   out_1747102118856192496[236] = 0.0;
   out_1747102118856192496[237] = 0.0;
   out_1747102118856192496[238] = 0.0;
   out_1747102118856192496[239] = 0.0;
   out_1747102118856192496[240] = 0.0;
   out_1747102118856192496[241] = 0.0;
   out_1747102118856192496[242] = 0.0;
   out_1747102118856192496[243] = 0.0;
   out_1747102118856192496[244] = 0.0;
   out_1747102118856192496[245] = 0.0;
   out_1747102118856192496[246] = 0.0;
   out_1747102118856192496[247] = 1.0;
   out_1747102118856192496[248] = 0.0;
   out_1747102118856192496[249] = 0.0;
   out_1747102118856192496[250] = 0.0;
   out_1747102118856192496[251] = 0.0;
   out_1747102118856192496[252] = 0.0;
   out_1747102118856192496[253] = 0.0;
   out_1747102118856192496[254] = 0.0;
   out_1747102118856192496[255] = 0.0;
   out_1747102118856192496[256] = 0.0;
   out_1747102118856192496[257] = 0.0;
   out_1747102118856192496[258] = 0.0;
   out_1747102118856192496[259] = 0.0;
   out_1747102118856192496[260] = 0.0;
   out_1747102118856192496[261] = 0.0;
   out_1747102118856192496[262] = 0.0;
   out_1747102118856192496[263] = 0.0;
   out_1747102118856192496[264] = 0.0;
   out_1747102118856192496[265] = 0.0;
   out_1747102118856192496[266] = 1.0;
   out_1747102118856192496[267] = 0.0;
   out_1747102118856192496[268] = 0.0;
   out_1747102118856192496[269] = 0.0;
   out_1747102118856192496[270] = 0.0;
   out_1747102118856192496[271] = 0.0;
   out_1747102118856192496[272] = 0.0;
   out_1747102118856192496[273] = 0.0;
   out_1747102118856192496[274] = 0.0;
   out_1747102118856192496[275] = 0.0;
   out_1747102118856192496[276] = 0.0;
   out_1747102118856192496[277] = 0.0;
   out_1747102118856192496[278] = 0.0;
   out_1747102118856192496[279] = 0.0;
   out_1747102118856192496[280] = 0.0;
   out_1747102118856192496[281] = 0.0;
   out_1747102118856192496[282] = 0.0;
   out_1747102118856192496[283] = 0.0;
   out_1747102118856192496[284] = 0.0;
   out_1747102118856192496[285] = 1.0;
   out_1747102118856192496[286] = 0.0;
   out_1747102118856192496[287] = 0.0;
   out_1747102118856192496[288] = 0.0;
   out_1747102118856192496[289] = 0.0;
   out_1747102118856192496[290] = 0.0;
   out_1747102118856192496[291] = 0.0;
   out_1747102118856192496[292] = 0.0;
   out_1747102118856192496[293] = 0.0;
   out_1747102118856192496[294] = 0.0;
   out_1747102118856192496[295] = 0.0;
   out_1747102118856192496[296] = 0.0;
   out_1747102118856192496[297] = 0.0;
   out_1747102118856192496[298] = 0.0;
   out_1747102118856192496[299] = 0.0;
   out_1747102118856192496[300] = 0.0;
   out_1747102118856192496[301] = 0.0;
   out_1747102118856192496[302] = 0.0;
   out_1747102118856192496[303] = 0.0;
   out_1747102118856192496[304] = 1.0;
   out_1747102118856192496[305] = 0.0;
   out_1747102118856192496[306] = 0.0;
   out_1747102118856192496[307] = 0.0;
   out_1747102118856192496[308] = 0.0;
   out_1747102118856192496[309] = 0.0;
   out_1747102118856192496[310] = 0.0;
   out_1747102118856192496[311] = 0.0;
   out_1747102118856192496[312] = 0.0;
   out_1747102118856192496[313] = 0.0;
   out_1747102118856192496[314] = 0.0;
   out_1747102118856192496[315] = 0.0;
   out_1747102118856192496[316] = 0.0;
   out_1747102118856192496[317] = 0.0;
   out_1747102118856192496[318] = 0.0;
   out_1747102118856192496[319] = 0.0;
   out_1747102118856192496[320] = 0.0;
   out_1747102118856192496[321] = 0.0;
   out_1747102118856192496[322] = 0.0;
   out_1747102118856192496[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7088515702411044915) {
   out_7088515702411044915[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7088515702411044915[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7088515702411044915[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7088515702411044915[3] = dt*state[12] + state[3];
   out_7088515702411044915[4] = dt*state[13] + state[4];
   out_7088515702411044915[5] = dt*state[14] + state[5];
   out_7088515702411044915[6] = state[6];
   out_7088515702411044915[7] = state[7];
   out_7088515702411044915[8] = state[8];
   out_7088515702411044915[9] = state[9];
   out_7088515702411044915[10] = state[10];
   out_7088515702411044915[11] = state[11];
   out_7088515702411044915[12] = state[12];
   out_7088515702411044915[13] = state[13];
   out_7088515702411044915[14] = state[14];
   out_7088515702411044915[15] = state[15];
   out_7088515702411044915[16] = state[16];
   out_7088515702411044915[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1598216342666213795) {
   out_1598216342666213795[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1598216342666213795[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1598216342666213795[2] = 0;
   out_1598216342666213795[3] = 0;
   out_1598216342666213795[4] = 0;
   out_1598216342666213795[5] = 0;
   out_1598216342666213795[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1598216342666213795[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1598216342666213795[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1598216342666213795[9] = 0;
   out_1598216342666213795[10] = 0;
   out_1598216342666213795[11] = 0;
   out_1598216342666213795[12] = 0;
   out_1598216342666213795[13] = 0;
   out_1598216342666213795[14] = 0;
   out_1598216342666213795[15] = 0;
   out_1598216342666213795[16] = 0;
   out_1598216342666213795[17] = 0;
   out_1598216342666213795[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1598216342666213795[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1598216342666213795[20] = 0;
   out_1598216342666213795[21] = 0;
   out_1598216342666213795[22] = 0;
   out_1598216342666213795[23] = 0;
   out_1598216342666213795[24] = 0;
   out_1598216342666213795[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1598216342666213795[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1598216342666213795[27] = 0;
   out_1598216342666213795[28] = 0;
   out_1598216342666213795[29] = 0;
   out_1598216342666213795[30] = 0;
   out_1598216342666213795[31] = 0;
   out_1598216342666213795[32] = 0;
   out_1598216342666213795[33] = 0;
   out_1598216342666213795[34] = 0;
   out_1598216342666213795[35] = 0;
   out_1598216342666213795[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1598216342666213795[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1598216342666213795[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1598216342666213795[39] = 0;
   out_1598216342666213795[40] = 0;
   out_1598216342666213795[41] = 0;
   out_1598216342666213795[42] = 0;
   out_1598216342666213795[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1598216342666213795[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1598216342666213795[45] = 0;
   out_1598216342666213795[46] = 0;
   out_1598216342666213795[47] = 0;
   out_1598216342666213795[48] = 0;
   out_1598216342666213795[49] = 0;
   out_1598216342666213795[50] = 0;
   out_1598216342666213795[51] = 0;
   out_1598216342666213795[52] = 0;
   out_1598216342666213795[53] = 0;
   out_1598216342666213795[54] = 0;
   out_1598216342666213795[55] = 0;
   out_1598216342666213795[56] = 0;
   out_1598216342666213795[57] = 1;
   out_1598216342666213795[58] = 0;
   out_1598216342666213795[59] = 0;
   out_1598216342666213795[60] = 0;
   out_1598216342666213795[61] = 0;
   out_1598216342666213795[62] = 0;
   out_1598216342666213795[63] = 0;
   out_1598216342666213795[64] = 0;
   out_1598216342666213795[65] = 0;
   out_1598216342666213795[66] = dt;
   out_1598216342666213795[67] = 0;
   out_1598216342666213795[68] = 0;
   out_1598216342666213795[69] = 0;
   out_1598216342666213795[70] = 0;
   out_1598216342666213795[71] = 0;
   out_1598216342666213795[72] = 0;
   out_1598216342666213795[73] = 0;
   out_1598216342666213795[74] = 0;
   out_1598216342666213795[75] = 0;
   out_1598216342666213795[76] = 1;
   out_1598216342666213795[77] = 0;
   out_1598216342666213795[78] = 0;
   out_1598216342666213795[79] = 0;
   out_1598216342666213795[80] = 0;
   out_1598216342666213795[81] = 0;
   out_1598216342666213795[82] = 0;
   out_1598216342666213795[83] = 0;
   out_1598216342666213795[84] = 0;
   out_1598216342666213795[85] = dt;
   out_1598216342666213795[86] = 0;
   out_1598216342666213795[87] = 0;
   out_1598216342666213795[88] = 0;
   out_1598216342666213795[89] = 0;
   out_1598216342666213795[90] = 0;
   out_1598216342666213795[91] = 0;
   out_1598216342666213795[92] = 0;
   out_1598216342666213795[93] = 0;
   out_1598216342666213795[94] = 0;
   out_1598216342666213795[95] = 1;
   out_1598216342666213795[96] = 0;
   out_1598216342666213795[97] = 0;
   out_1598216342666213795[98] = 0;
   out_1598216342666213795[99] = 0;
   out_1598216342666213795[100] = 0;
   out_1598216342666213795[101] = 0;
   out_1598216342666213795[102] = 0;
   out_1598216342666213795[103] = 0;
   out_1598216342666213795[104] = dt;
   out_1598216342666213795[105] = 0;
   out_1598216342666213795[106] = 0;
   out_1598216342666213795[107] = 0;
   out_1598216342666213795[108] = 0;
   out_1598216342666213795[109] = 0;
   out_1598216342666213795[110] = 0;
   out_1598216342666213795[111] = 0;
   out_1598216342666213795[112] = 0;
   out_1598216342666213795[113] = 0;
   out_1598216342666213795[114] = 1;
   out_1598216342666213795[115] = 0;
   out_1598216342666213795[116] = 0;
   out_1598216342666213795[117] = 0;
   out_1598216342666213795[118] = 0;
   out_1598216342666213795[119] = 0;
   out_1598216342666213795[120] = 0;
   out_1598216342666213795[121] = 0;
   out_1598216342666213795[122] = 0;
   out_1598216342666213795[123] = 0;
   out_1598216342666213795[124] = 0;
   out_1598216342666213795[125] = 0;
   out_1598216342666213795[126] = 0;
   out_1598216342666213795[127] = 0;
   out_1598216342666213795[128] = 0;
   out_1598216342666213795[129] = 0;
   out_1598216342666213795[130] = 0;
   out_1598216342666213795[131] = 0;
   out_1598216342666213795[132] = 0;
   out_1598216342666213795[133] = 1;
   out_1598216342666213795[134] = 0;
   out_1598216342666213795[135] = 0;
   out_1598216342666213795[136] = 0;
   out_1598216342666213795[137] = 0;
   out_1598216342666213795[138] = 0;
   out_1598216342666213795[139] = 0;
   out_1598216342666213795[140] = 0;
   out_1598216342666213795[141] = 0;
   out_1598216342666213795[142] = 0;
   out_1598216342666213795[143] = 0;
   out_1598216342666213795[144] = 0;
   out_1598216342666213795[145] = 0;
   out_1598216342666213795[146] = 0;
   out_1598216342666213795[147] = 0;
   out_1598216342666213795[148] = 0;
   out_1598216342666213795[149] = 0;
   out_1598216342666213795[150] = 0;
   out_1598216342666213795[151] = 0;
   out_1598216342666213795[152] = 1;
   out_1598216342666213795[153] = 0;
   out_1598216342666213795[154] = 0;
   out_1598216342666213795[155] = 0;
   out_1598216342666213795[156] = 0;
   out_1598216342666213795[157] = 0;
   out_1598216342666213795[158] = 0;
   out_1598216342666213795[159] = 0;
   out_1598216342666213795[160] = 0;
   out_1598216342666213795[161] = 0;
   out_1598216342666213795[162] = 0;
   out_1598216342666213795[163] = 0;
   out_1598216342666213795[164] = 0;
   out_1598216342666213795[165] = 0;
   out_1598216342666213795[166] = 0;
   out_1598216342666213795[167] = 0;
   out_1598216342666213795[168] = 0;
   out_1598216342666213795[169] = 0;
   out_1598216342666213795[170] = 0;
   out_1598216342666213795[171] = 1;
   out_1598216342666213795[172] = 0;
   out_1598216342666213795[173] = 0;
   out_1598216342666213795[174] = 0;
   out_1598216342666213795[175] = 0;
   out_1598216342666213795[176] = 0;
   out_1598216342666213795[177] = 0;
   out_1598216342666213795[178] = 0;
   out_1598216342666213795[179] = 0;
   out_1598216342666213795[180] = 0;
   out_1598216342666213795[181] = 0;
   out_1598216342666213795[182] = 0;
   out_1598216342666213795[183] = 0;
   out_1598216342666213795[184] = 0;
   out_1598216342666213795[185] = 0;
   out_1598216342666213795[186] = 0;
   out_1598216342666213795[187] = 0;
   out_1598216342666213795[188] = 0;
   out_1598216342666213795[189] = 0;
   out_1598216342666213795[190] = 1;
   out_1598216342666213795[191] = 0;
   out_1598216342666213795[192] = 0;
   out_1598216342666213795[193] = 0;
   out_1598216342666213795[194] = 0;
   out_1598216342666213795[195] = 0;
   out_1598216342666213795[196] = 0;
   out_1598216342666213795[197] = 0;
   out_1598216342666213795[198] = 0;
   out_1598216342666213795[199] = 0;
   out_1598216342666213795[200] = 0;
   out_1598216342666213795[201] = 0;
   out_1598216342666213795[202] = 0;
   out_1598216342666213795[203] = 0;
   out_1598216342666213795[204] = 0;
   out_1598216342666213795[205] = 0;
   out_1598216342666213795[206] = 0;
   out_1598216342666213795[207] = 0;
   out_1598216342666213795[208] = 0;
   out_1598216342666213795[209] = 1;
   out_1598216342666213795[210] = 0;
   out_1598216342666213795[211] = 0;
   out_1598216342666213795[212] = 0;
   out_1598216342666213795[213] = 0;
   out_1598216342666213795[214] = 0;
   out_1598216342666213795[215] = 0;
   out_1598216342666213795[216] = 0;
   out_1598216342666213795[217] = 0;
   out_1598216342666213795[218] = 0;
   out_1598216342666213795[219] = 0;
   out_1598216342666213795[220] = 0;
   out_1598216342666213795[221] = 0;
   out_1598216342666213795[222] = 0;
   out_1598216342666213795[223] = 0;
   out_1598216342666213795[224] = 0;
   out_1598216342666213795[225] = 0;
   out_1598216342666213795[226] = 0;
   out_1598216342666213795[227] = 0;
   out_1598216342666213795[228] = 1;
   out_1598216342666213795[229] = 0;
   out_1598216342666213795[230] = 0;
   out_1598216342666213795[231] = 0;
   out_1598216342666213795[232] = 0;
   out_1598216342666213795[233] = 0;
   out_1598216342666213795[234] = 0;
   out_1598216342666213795[235] = 0;
   out_1598216342666213795[236] = 0;
   out_1598216342666213795[237] = 0;
   out_1598216342666213795[238] = 0;
   out_1598216342666213795[239] = 0;
   out_1598216342666213795[240] = 0;
   out_1598216342666213795[241] = 0;
   out_1598216342666213795[242] = 0;
   out_1598216342666213795[243] = 0;
   out_1598216342666213795[244] = 0;
   out_1598216342666213795[245] = 0;
   out_1598216342666213795[246] = 0;
   out_1598216342666213795[247] = 1;
   out_1598216342666213795[248] = 0;
   out_1598216342666213795[249] = 0;
   out_1598216342666213795[250] = 0;
   out_1598216342666213795[251] = 0;
   out_1598216342666213795[252] = 0;
   out_1598216342666213795[253] = 0;
   out_1598216342666213795[254] = 0;
   out_1598216342666213795[255] = 0;
   out_1598216342666213795[256] = 0;
   out_1598216342666213795[257] = 0;
   out_1598216342666213795[258] = 0;
   out_1598216342666213795[259] = 0;
   out_1598216342666213795[260] = 0;
   out_1598216342666213795[261] = 0;
   out_1598216342666213795[262] = 0;
   out_1598216342666213795[263] = 0;
   out_1598216342666213795[264] = 0;
   out_1598216342666213795[265] = 0;
   out_1598216342666213795[266] = 1;
   out_1598216342666213795[267] = 0;
   out_1598216342666213795[268] = 0;
   out_1598216342666213795[269] = 0;
   out_1598216342666213795[270] = 0;
   out_1598216342666213795[271] = 0;
   out_1598216342666213795[272] = 0;
   out_1598216342666213795[273] = 0;
   out_1598216342666213795[274] = 0;
   out_1598216342666213795[275] = 0;
   out_1598216342666213795[276] = 0;
   out_1598216342666213795[277] = 0;
   out_1598216342666213795[278] = 0;
   out_1598216342666213795[279] = 0;
   out_1598216342666213795[280] = 0;
   out_1598216342666213795[281] = 0;
   out_1598216342666213795[282] = 0;
   out_1598216342666213795[283] = 0;
   out_1598216342666213795[284] = 0;
   out_1598216342666213795[285] = 1;
   out_1598216342666213795[286] = 0;
   out_1598216342666213795[287] = 0;
   out_1598216342666213795[288] = 0;
   out_1598216342666213795[289] = 0;
   out_1598216342666213795[290] = 0;
   out_1598216342666213795[291] = 0;
   out_1598216342666213795[292] = 0;
   out_1598216342666213795[293] = 0;
   out_1598216342666213795[294] = 0;
   out_1598216342666213795[295] = 0;
   out_1598216342666213795[296] = 0;
   out_1598216342666213795[297] = 0;
   out_1598216342666213795[298] = 0;
   out_1598216342666213795[299] = 0;
   out_1598216342666213795[300] = 0;
   out_1598216342666213795[301] = 0;
   out_1598216342666213795[302] = 0;
   out_1598216342666213795[303] = 0;
   out_1598216342666213795[304] = 1;
   out_1598216342666213795[305] = 0;
   out_1598216342666213795[306] = 0;
   out_1598216342666213795[307] = 0;
   out_1598216342666213795[308] = 0;
   out_1598216342666213795[309] = 0;
   out_1598216342666213795[310] = 0;
   out_1598216342666213795[311] = 0;
   out_1598216342666213795[312] = 0;
   out_1598216342666213795[313] = 0;
   out_1598216342666213795[314] = 0;
   out_1598216342666213795[315] = 0;
   out_1598216342666213795[316] = 0;
   out_1598216342666213795[317] = 0;
   out_1598216342666213795[318] = 0;
   out_1598216342666213795[319] = 0;
   out_1598216342666213795[320] = 0;
   out_1598216342666213795[321] = 0;
   out_1598216342666213795[322] = 0;
   out_1598216342666213795[323] = 1;
}
void h_4(double *state, double *unused, double *out_5322908657184251983) {
   out_5322908657184251983[0] = state[6] + state[9];
   out_5322908657184251983[1] = state[7] + state[10];
   out_5322908657184251983[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2501263120910905103) {
   out_2501263120910905103[0] = 0;
   out_2501263120910905103[1] = 0;
   out_2501263120910905103[2] = 0;
   out_2501263120910905103[3] = 0;
   out_2501263120910905103[4] = 0;
   out_2501263120910905103[5] = 0;
   out_2501263120910905103[6] = 1;
   out_2501263120910905103[7] = 0;
   out_2501263120910905103[8] = 0;
   out_2501263120910905103[9] = 1;
   out_2501263120910905103[10] = 0;
   out_2501263120910905103[11] = 0;
   out_2501263120910905103[12] = 0;
   out_2501263120910905103[13] = 0;
   out_2501263120910905103[14] = 0;
   out_2501263120910905103[15] = 0;
   out_2501263120910905103[16] = 0;
   out_2501263120910905103[17] = 0;
   out_2501263120910905103[18] = 0;
   out_2501263120910905103[19] = 0;
   out_2501263120910905103[20] = 0;
   out_2501263120910905103[21] = 0;
   out_2501263120910905103[22] = 0;
   out_2501263120910905103[23] = 0;
   out_2501263120910905103[24] = 0;
   out_2501263120910905103[25] = 1;
   out_2501263120910905103[26] = 0;
   out_2501263120910905103[27] = 0;
   out_2501263120910905103[28] = 1;
   out_2501263120910905103[29] = 0;
   out_2501263120910905103[30] = 0;
   out_2501263120910905103[31] = 0;
   out_2501263120910905103[32] = 0;
   out_2501263120910905103[33] = 0;
   out_2501263120910905103[34] = 0;
   out_2501263120910905103[35] = 0;
   out_2501263120910905103[36] = 0;
   out_2501263120910905103[37] = 0;
   out_2501263120910905103[38] = 0;
   out_2501263120910905103[39] = 0;
   out_2501263120910905103[40] = 0;
   out_2501263120910905103[41] = 0;
   out_2501263120910905103[42] = 0;
   out_2501263120910905103[43] = 0;
   out_2501263120910905103[44] = 1;
   out_2501263120910905103[45] = 0;
   out_2501263120910905103[46] = 0;
   out_2501263120910905103[47] = 1;
   out_2501263120910905103[48] = 0;
   out_2501263120910905103[49] = 0;
   out_2501263120910905103[50] = 0;
   out_2501263120910905103[51] = 0;
   out_2501263120910905103[52] = 0;
   out_2501263120910905103[53] = 0;
}
void h_10(double *state, double *unused, double *out_3029307404736649758) {
   out_3029307404736649758[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3029307404736649758[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3029307404736649758[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3781124612136597110) {
   out_3781124612136597110[0] = 0;
   out_3781124612136597110[1] = 9.8100000000000005*cos(state[1]);
   out_3781124612136597110[2] = 0;
   out_3781124612136597110[3] = 0;
   out_3781124612136597110[4] = -state[8];
   out_3781124612136597110[5] = state[7];
   out_3781124612136597110[6] = 0;
   out_3781124612136597110[7] = state[5];
   out_3781124612136597110[8] = -state[4];
   out_3781124612136597110[9] = 0;
   out_3781124612136597110[10] = 0;
   out_3781124612136597110[11] = 0;
   out_3781124612136597110[12] = 1;
   out_3781124612136597110[13] = 0;
   out_3781124612136597110[14] = 0;
   out_3781124612136597110[15] = 1;
   out_3781124612136597110[16] = 0;
   out_3781124612136597110[17] = 0;
   out_3781124612136597110[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3781124612136597110[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3781124612136597110[20] = 0;
   out_3781124612136597110[21] = state[8];
   out_3781124612136597110[22] = 0;
   out_3781124612136597110[23] = -state[6];
   out_3781124612136597110[24] = -state[5];
   out_3781124612136597110[25] = 0;
   out_3781124612136597110[26] = state[3];
   out_3781124612136597110[27] = 0;
   out_3781124612136597110[28] = 0;
   out_3781124612136597110[29] = 0;
   out_3781124612136597110[30] = 0;
   out_3781124612136597110[31] = 1;
   out_3781124612136597110[32] = 0;
   out_3781124612136597110[33] = 0;
   out_3781124612136597110[34] = 1;
   out_3781124612136597110[35] = 0;
   out_3781124612136597110[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3781124612136597110[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3781124612136597110[38] = 0;
   out_3781124612136597110[39] = -state[7];
   out_3781124612136597110[40] = state[6];
   out_3781124612136597110[41] = 0;
   out_3781124612136597110[42] = state[4];
   out_3781124612136597110[43] = -state[3];
   out_3781124612136597110[44] = 0;
   out_3781124612136597110[45] = 0;
   out_3781124612136597110[46] = 0;
   out_3781124612136597110[47] = 0;
   out_3781124612136597110[48] = 0;
   out_3781124612136597110[49] = 0;
   out_3781124612136597110[50] = 1;
   out_3781124612136597110[51] = 0;
   out_3781124612136597110[52] = 0;
   out_3781124612136597110[53] = 1;
}
void h_13(double *state, double *unused, double *out_1688939920546120950) {
   out_1688939920546120950[0] = state[3];
   out_1688939920546120950[1] = state[4];
   out_1688939920546120950[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8334849744481945584) {
   out_8334849744481945584[0] = 0;
   out_8334849744481945584[1] = 0;
   out_8334849744481945584[2] = 0;
   out_8334849744481945584[3] = 1;
   out_8334849744481945584[4] = 0;
   out_8334849744481945584[5] = 0;
   out_8334849744481945584[6] = 0;
   out_8334849744481945584[7] = 0;
   out_8334849744481945584[8] = 0;
   out_8334849744481945584[9] = 0;
   out_8334849744481945584[10] = 0;
   out_8334849744481945584[11] = 0;
   out_8334849744481945584[12] = 0;
   out_8334849744481945584[13] = 0;
   out_8334849744481945584[14] = 0;
   out_8334849744481945584[15] = 0;
   out_8334849744481945584[16] = 0;
   out_8334849744481945584[17] = 0;
   out_8334849744481945584[18] = 0;
   out_8334849744481945584[19] = 0;
   out_8334849744481945584[20] = 0;
   out_8334849744481945584[21] = 0;
   out_8334849744481945584[22] = 1;
   out_8334849744481945584[23] = 0;
   out_8334849744481945584[24] = 0;
   out_8334849744481945584[25] = 0;
   out_8334849744481945584[26] = 0;
   out_8334849744481945584[27] = 0;
   out_8334849744481945584[28] = 0;
   out_8334849744481945584[29] = 0;
   out_8334849744481945584[30] = 0;
   out_8334849744481945584[31] = 0;
   out_8334849744481945584[32] = 0;
   out_8334849744481945584[33] = 0;
   out_8334849744481945584[34] = 0;
   out_8334849744481945584[35] = 0;
   out_8334849744481945584[36] = 0;
   out_8334849744481945584[37] = 0;
   out_8334849744481945584[38] = 0;
   out_8334849744481945584[39] = 0;
   out_8334849744481945584[40] = 0;
   out_8334849744481945584[41] = 1;
   out_8334849744481945584[42] = 0;
   out_8334849744481945584[43] = 0;
   out_8334849744481945584[44] = 0;
   out_8334849744481945584[45] = 0;
   out_8334849744481945584[46] = 0;
   out_8334849744481945584[47] = 0;
   out_8334849744481945584[48] = 0;
   out_8334849744481945584[49] = 0;
   out_8334849744481945584[50] = 0;
   out_8334849744481945584[51] = 0;
   out_8334849744481945584[52] = 0;
   out_8334849744481945584[53] = 0;
}
void h_14(double *state, double *unused, double *out_6837219853172441902) {
   out_6837219853172441902[0] = state[6];
   out_6837219853172441902[1] = state[7];
   out_6837219853172441902[2] = state[8];
}
void H_14(double *state, double *unused, double *out_581525311384467193) {
   out_581525311384467193[0] = 0;
   out_581525311384467193[1] = 0;
   out_581525311384467193[2] = 0;
   out_581525311384467193[3] = 0;
   out_581525311384467193[4] = 0;
   out_581525311384467193[5] = 0;
   out_581525311384467193[6] = 1;
   out_581525311384467193[7] = 0;
   out_581525311384467193[8] = 0;
   out_581525311384467193[9] = 0;
   out_581525311384467193[10] = 0;
   out_581525311384467193[11] = 0;
   out_581525311384467193[12] = 0;
   out_581525311384467193[13] = 0;
   out_581525311384467193[14] = 0;
   out_581525311384467193[15] = 0;
   out_581525311384467193[16] = 0;
   out_581525311384467193[17] = 0;
   out_581525311384467193[18] = 0;
   out_581525311384467193[19] = 0;
   out_581525311384467193[20] = 0;
   out_581525311384467193[21] = 0;
   out_581525311384467193[22] = 0;
   out_581525311384467193[23] = 0;
   out_581525311384467193[24] = 0;
   out_581525311384467193[25] = 1;
   out_581525311384467193[26] = 0;
   out_581525311384467193[27] = 0;
   out_581525311384467193[28] = 0;
   out_581525311384467193[29] = 0;
   out_581525311384467193[30] = 0;
   out_581525311384467193[31] = 0;
   out_581525311384467193[32] = 0;
   out_581525311384467193[33] = 0;
   out_581525311384467193[34] = 0;
   out_581525311384467193[35] = 0;
   out_581525311384467193[36] = 0;
   out_581525311384467193[37] = 0;
   out_581525311384467193[38] = 0;
   out_581525311384467193[39] = 0;
   out_581525311384467193[40] = 0;
   out_581525311384467193[41] = 0;
   out_581525311384467193[42] = 0;
   out_581525311384467193[43] = 0;
   out_581525311384467193[44] = 1;
   out_581525311384467193[45] = 0;
   out_581525311384467193[46] = 0;
   out_581525311384467193[47] = 0;
   out_581525311384467193[48] = 0;
   out_581525311384467193[49] = 0;
   out_581525311384467193[50] = 0;
   out_581525311384467193[51] = 0;
   out_581525311384467193[52] = 0;
   out_581525311384467193[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2768303435411887346) {
  err_fun(nom_x, delta_x, out_2768303435411887346);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1488408519072955526) {
  inv_err_fun(nom_x, true_x, out_1488408519072955526);
}
void pose_H_mod_fun(double *state, double *out_1747102118856192496) {
  H_mod_fun(state, out_1747102118856192496);
}
void pose_f_fun(double *state, double dt, double *out_7088515702411044915) {
  f_fun(state,  dt, out_7088515702411044915);
}
void pose_F_fun(double *state, double dt, double *out_1598216342666213795) {
  F_fun(state,  dt, out_1598216342666213795);
}
void pose_h_4(double *state, double *unused, double *out_5322908657184251983) {
  h_4(state, unused, out_5322908657184251983);
}
void pose_H_4(double *state, double *unused, double *out_2501263120910905103) {
  H_4(state, unused, out_2501263120910905103);
}
void pose_h_10(double *state, double *unused, double *out_3029307404736649758) {
  h_10(state, unused, out_3029307404736649758);
}
void pose_H_10(double *state, double *unused, double *out_3781124612136597110) {
  H_10(state, unused, out_3781124612136597110);
}
void pose_h_13(double *state, double *unused, double *out_1688939920546120950) {
  h_13(state, unused, out_1688939920546120950);
}
void pose_H_13(double *state, double *unused, double *out_8334849744481945584) {
  H_13(state, unused, out_8334849744481945584);
}
void pose_h_14(double *state, double *unused, double *out_6837219853172441902) {
  h_14(state, unused, out_6837219853172441902);
}
void pose_H_14(double *state, double *unused, double *out_581525311384467193) {
  H_14(state, unused, out_581525311384467193);
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
