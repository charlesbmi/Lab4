// Custom IROM with self-playing Pong

`include "mips_defines.v"

`define ADDR_WIDTH 9
`define INSTR_WIDTH 32
`define NUM_INSTR 512

module irom(addr, dout);
  input [`ADDR_WIDTH-1:0] addr;
  output wire [`INSTR_WIDTH-1:0] dout;

  wire [`INSTR_WIDTH-1:0] memory [`NUM_INSTR-1:0];

  assign dout = memory[addr];

  assign memory[  0] = {`LUI, `ZERO, `T0, 16'hffff};
  assign memory[  1] = {`ORI, `T0, `T0, 16'h000c};
  assign memory[  2] = {`LUI, `NULL, `T1, 16'h6};
  assign memory[  3] = {`ORI, `T1, `T1, 8'd1, 8'd1};
  assign memory[  4] = {`SW, `T0, `T1, 16'd0};
  assign memory[  5] = {`LUI, `NULL, `T1, 16'h4};
  assign memory[  6] = {`ORI, `T1, `T1, 8'd1, 8'd2}; 
  assign memory[  7] = {`SW, `T0, `T1, 16'd0};
  assign memory[  8] = {`LUI, `NULL, `T1, 16'h1};
  assign memory[  9] = {`ORI, `T1, `T1, 8'd1, 8'd3};
  assign memory[ 10] = {`LUI, `NULL, `T6, 16'b011};
  assign memory[ 11] = {`ADDI, `T6, `T6, 8'd11, 8'd3};
  assign memory[ 12] = {`BEQ, `T1, `T6, 16'd24};
  assign memory[ 13] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 14] = {`NOP};
  assign memory[ 15] = {`NOP};
  assign memory[ 16] = {`LUI, `NULL, `T1, 16'h2};
  assign memory[ 17] = {`ORI, `T1, `T1, 8'd10, 8'd1};
  assign memory[ 18] = {`NOP};
  assign memory[ 19] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 20] = {`LUI, `NULL, `T1, 16'h5};
  assign memory[ 21] = {`ORI, `T1, `T1, 8'd10, 8'd2};
  assign memory[ 22] = {`LW, `T0, `S0, 16'd0};
  assign memory[ 23] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 24] = {`LUI, `NULL, `T1, 16'h3};
  assign memory[ 25] = {`ORI, `T1, `T1, 8'd10, 8'd3};
  assign memory[ 26] = {`NOP};
  assign memory[ 27] = {`SW, `T0, `T1, 16'd0};       
  assign memory[ 28] = {`ADDI, `ZERO, `T2, 16'd36};
  assign memory[ 29] = {`SPECIAL, `T2, `NULL, `NULL, `NULL, `JR};
  assign memory[ 30] = {`J, 26'd40};
  assign memory[ 31] = {`NOP};
  assign memory[ 32] = {`LUI, `NULL, `T1, 16'h3};
  assign memory[ 33] = {`ORI, `T1, `T1, 8'd20, 8'd1};
  assign memory[ 34] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 35] = {`NOP};
  assign memory[ 36] = {`LUI, `NULL, `T1, 16'h2};
  assign memory[ 37] = {`ORI, `T1, `T1, 8'd20, 8'd2};
  assign memory[ 38] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 39] = {`NOP};
  assign memory[ 40] = {`LUI, `NULL, `T1, 16'h5};
  assign memory[ 41] = {`ORI, `T1, `T1, 8'd20, 8'd3};
  assign memory[ 42] = {`LW, `T0, `S0, 16'd0};
  assign memory[ 43] = {`SW, `T0, `T1, 16'd0};       
  assign memory[ 44] = {`LW, `T0, `S0, 16'd0};
  assign memory[ 45] = {`LW, `T0, `S0, 16'd0};
  assign memory[ 46] = {`NOP};
  assign memory[ 47] = {`NOP};
  assign memory[ 48] = {`NOP};
  assign memory[ 49] = {`NOP};
  assign memory[ 50] = {`LUI, `NULL, `T5, 16'b010};
  assign memory[ 51] = {`ORI, `T5, `T5, 8'd28, 8'd2};
  assign memory[ 52] = {`SW, `ZERO, `T5, 16'd40};
  assign memory[ 53] = {`LW, `ZERO, `T7, 16'd40};
  assign memory[ 54] = {`SW, `T0, `T7, 16'd0};
  assign memory[ 55] = {`NOP};
  assign memory[ 56] = {`NOP};
  assign memory[ 57] = {`NOP};
  assign memory[ 58] = {`LUI, `NULL, `T8, 16'b011};
  assign memory[ 59] = {`ORI, `T8, `T8, {8'd39, 8'd0}};
  assign memory[ 60] = {`BLEZ, `T8, `NULL, 16'd1};
  assign memory[ 61] = {`NOP};
  assign memory[ 62] = {`ORI, `T8, `T8, {8'd0, 8'd1}}; // should run
  assign memory[ 63] = {`LUI, `ZERO, `T0, 16'hffff};
  assign memory[ 64] = {`ORI, `T0, `T0, 16'h000c};
  assign memory[ 65] = {`ADDI, `ZERO, `A0, 16'd0};
  assign memory[ 66] = {`ADDI, `ZERO, `A1, 16'd0};
  assign memory[ 67] = {`ADDI, `ZERO, `A2, 16'd0};
  assign memory[ 68] = {`ADDI, `ZERO, `A3, 16'd0};
  assign memory[ 69] = {`NOP};
  assign memory[ 70] = {`ADDI, `ZERO, `S0, 16'd15};
  assign memory[ 71] = {`ADDI, `ZERO, `S1, 16'd12};
  assign memory[ 72] = {`JAL, 26'd101};
  assign memory[ 73] = {`LUI, `NULL, `T1, 16'h6};
  assign memory[ 74] = {`ORI, `T1, `T1, 8'd1, 8'd1};
  assign memory[ 75] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 76] = {`NOP};
  assign memory[ 77] = {`JAL, 26'd104};
  assign memory[ 78] = {`LUI, `NULL, `T1, 16'h4};
  assign memory[ 79] = {`ORI, `T1, `T1, 8'd1, 8'd2};
  assign memory[ 80] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 81] = {`NOP};
  assign memory[ 82] = {`JAL, 26'd106};
  assign memory[ 83] = {`LUI, `NULL, `T1, 16'h1};
  assign memory[ 84] = {`ORI, `T1, `T1, 8'd1, 8'd3};
  assign memory[ 85] = {`SW, `T0, `T1, 16'd0};
  assign memory[ 86] = {`NOP};
  assign memory[ 87] = {`NOP};
  assign memory[ 88] = {`NOP};
  assign memory[ 89] = {`NOP};
  assign memory[ 90] = {`NOP};
  assign memory[ 91] = {`NOP};
  assign memory[ 92] = {`NOP};
  assign memory[ 93] = {`NOP};
  assign memory[ 94] = {`NOP};
  assign memory[ 95] = {`NOP};
  assign memory[ 96] = {`NOP};
  assign memory[ 97] = {`NOP};
  assign memory[ 98] = {`NOP};
  assign memory[ 99] = {`NOP};
  assign memory[100] = {`NOP}; // say A0 contains the address you want to write to, A1 contains one value, A2 contains second value
  assign memory[101] = {`NOP}; // write miss
  assign memory[102] = {`NOP}; // read miss
  assign memory[103] = {`NOP}; // check that right value is read
  assign memory[104] = {`NOP}; // read hit
  assign memory[105] = {`NOP}; // check that right value is read
  assign memory[106] = {`NOP}; // write hit (same location as write miss)
  assign memory[107] = {`NOP};
  assign memory[108] = {`NOP};
  assign memory[109] = {`NOP};
  assign memory[110] = {`NOP};
  assign memory[111] = {`NOP};
  assign memory[112] = {`NOP};
  assign memory[113] = {`NOP};
  assign memory[114] = {`NOP};
  assign memory[115] = {`NOP};
  assign memory[116] = {`NOP};
  assign memory[117] = {`NOP};
  assign memory[118] = {`NOP};
  assign memory[119] = {`NOP};
  assign memory[120] = {`NOP};
  assign memory[121] = {`NOP};
  assign memory[122] = {`NOP};
  assign memory[123] = {`NOP};
  assign memory[124] = {`NOP};
  assign memory[125] = {`NOP};
  assign memory[126] = {`NOP};
  assign memory[127] = {`NOP};
  assign memory[128] = {`NOP};
  assign memory[129] = {`NOP};
  assign memory[130] = {`NOP};
  assign memory[131] = {`NOP};
  assign memory[132] = {`NOP};
  assign memory[133] = {`NOP};
  assign memory[134] = {`NOP};
  assign memory[135] = {`NOP};
  assign memory[136] = {`NOP};
  assign memory[137] = {`NOP};
  assign memory[138] = {`NOP};
  assign memory[139] = {`NOP};
  assign memory[140] = {`NOP};
  assign memory[141] = {`NOP};
  assign memory[142] = {`NOP};
  assign memory[143] = {`NOP};
  assign memory[144] = {`NOP};
  assign memory[145] = {`NOP};
  assign memory[146] = {`NOP};
  assign memory[147] = {`NOP};
  assign memory[148] = {`NOP};
  assign memory[149] = {`NOP};
  assign memory[150] = {`NOP};
  assign memory[151] = {`NOP};
  assign memory[152] = {`NOP};
  assign memory[153] = {`NOP};
  assign memory[154] = {`NOP};
  assign memory[155] = {`NOP};
  assign memory[156] = {`NOP};
  assign memory[157] = {`NOP};
  assign memory[158] = {`NOP};
  assign memory[159] = {`NOP};
  assign memory[160] = {`NOP};
  assign memory[161] = {`NOP};
  assign memory[162] = {`NOP};
  assign memory[163] = {`NOP};
  assign memory[164] = {`NOP};
  assign memory[165] = {`NOP};
  assign memory[166] = {`NOP};
  assign memory[167] = {`NOP};
  assign memory[168] = {`NOP};
  assign memory[169] = {`NOP};
  assign memory[170] = {`NOP};
  assign memory[171] = {`NOP};
  assign memory[172] = {`NOP};
  assign memory[173] = {`NOP};
  assign memory[174] = {`NOP};
  assign memory[175] = {`NOP};
  assign memory[176] = {`NOP};
  assign memory[177] = {`NOP};
  assign memory[178] = {`NOP};
  assign memory[179] = {`NOP};
  assign memory[180] = {`NOP};
  assign memory[181] = {`NOP};
  assign memory[182] = {`NOP};
  assign memory[183] = {`NOP};
  assign memory[184] = {`NOP};
  assign memory[185] = {`NOP};
  assign memory[186] = {`NOP};
  assign memory[187] = {`NOP};
  assign memory[188] = {`NOP};
  assign memory[189] = {`NOP};
  assign memory[190] = {`NOP};
  assign memory[191] = {`NOP};
  assign memory[192] = {`NOP};
  assign memory[193] = {`NOP};
  assign memory[194] = {`NOP};
  assign memory[195] = {`NOP};
  assign memory[196] = {`NOP};
  assign memory[197] = {`NOP};
  assign memory[198] = {`NOP};
  assign memory[199] = {`NOP};
  assign memory[200] = {`NOP};
  assign memory[201] = {`NOP};
  assign memory[202] = {`NOP};
  assign memory[203] = {`NOP};
  assign memory[204] = {`NOP};
  assign memory[205] = {`NOP};
  assign memory[206] = {`NOP};
  assign memory[207] = {`NOP};
  assign memory[208] = {`NOP};
  assign memory[209] = {`NOP};
  assign memory[210] = {`NOP};
  assign memory[211] = {`NOP};
  assign memory[212] = {`NOP};
  assign memory[213] = {`NOP};
  assign memory[214] = {`NOP};
  assign memory[215] = {`NOP};
  assign memory[216] = {`NOP};
  assign memory[217] = {`NOP};
  assign memory[218] = {`NOP};
  assign memory[219] = {`NOP};
  assign memory[220] = {`NOP};
  assign memory[221] = {`NOP};
  assign memory[222] = {`NOP};
  assign memory[223] = {`NOP};
  assign memory[224] = {`NOP};
  assign memory[225] = {`NOP};
  assign memory[226] = {`NOP};
  assign memory[227] = {`NOP};
  assign memory[228] = {`NOP};
  assign memory[229] = {`NOP};
  assign memory[230] = {`NOP};
  assign memory[231] = {`NOP};
  assign memory[232] = {`NOP};
  assign memory[233] = {`NOP};
  assign memory[234] = {`NOP};
  assign memory[235] = {`NOP};
  assign memory[236] = {`NOP};
  assign memory[237] = {`NOP};
  assign memory[238] = {`NOP};
  assign memory[239] = {`NOP};
  assign memory[240] = {`NOP};
  assign memory[241] = {`NOP};
  assign memory[242] = {`NOP};
  assign memory[243] = {`NOP};
  assign memory[244] = {`NOP};
  assign memory[245] = {`NOP};
  assign memory[246] = {`NOP};
  assign memory[247] = {`NOP};
  assign memory[248] = {`NOP};
  assign memory[249] = {`NOP};
  assign memory[250] = {`NOP};
  assign memory[251] = {`NOP};
  assign memory[252] = {`NOP};
  assign memory[253] = {`NOP};
  assign memory[254] = {`NOP};
  assign memory[255] = {`NOP};
  assign memory[256] = {`NOP};
  assign memory[257] = {`NOP};
  assign memory[258] = {`NOP};
  assign memory[259] = {`NOP};
  assign memory[260] = {`NOP};
  assign memory[261] = {`NOP};
  assign memory[262] = {`NOP};
  assign memory[263] = {`NOP};
  assign memory[264] = {`NOP};
  assign memory[265] = {`NOP};
  assign memory[266] = {`NOP};
  assign memory[267] = {`NOP};
  assign memory[268] = {`NOP};
  assign memory[269] = {`NOP};
  assign memory[270] = {`NOP};
  assign memory[271] = {`NOP};
  assign memory[272] = {`NOP};
  assign memory[273] = {`NOP};
  assign memory[274] = {`NOP};
  assign memory[275] = {`NOP};
  assign memory[276] = {`NOP};
  assign memory[277] = {`NOP};
  assign memory[278] = {`NOP};
  assign memory[279] = {`NOP};
  assign memory[280] = {`NOP};
  assign memory[281] = {`NOP};
  assign memory[282] = {`NOP};
  assign memory[283] = {`NOP};
  assign memory[284] = {`NOP};
  assign memory[285] = {`NOP};
  assign memory[286] = {`NOP};
  assign memory[287] = {`NOP};
  assign memory[288] = {`NOP};
  assign memory[289] = {`NOP};
  assign memory[290] = {`NOP};
  assign memory[291] = {`NOP};
  assign memory[292] = {`NOP};
  assign memory[293] = {`NOP};
  assign memory[294] = {`NOP};
  assign memory[295] = {`NOP};
  assign memory[296] = {`NOP};
  assign memory[297] = {`NOP};
  assign memory[298] = {`NOP};
  assign memory[299] = {`NOP};
  assign memory[300] = {`NOP};
  assign memory[301] = {`NOP};
  assign memory[302] = {`NOP};
  assign memory[303] = {`NOP};
  assign memory[304] = {`NOP};
  assign memory[305] = {`NOP};
  assign memory[306] = {`NOP};
  assign memory[307] = {`NOP};
  assign memory[308] = {`NOP};
  assign memory[309] = {`NOP};
  assign memory[310] = {`NOP};
  assign memory[311] = {`NOP};
  assign memory[312] = {`NOP};
  assign memory[313] = {`NOP};
  assign memory[314] = {`NOP};
  assign memory[315] = {`NOP};
  assign memory[316] = {`NOP};
  assign memory[317] = {`NOP};
  assign memory[318] = {`NOP};
  assign memory[319] = {`NOP};
  assign memory[320] = {`NOP};
  assign memory[321] = {`NOP};
  assign memory[322] = {`NOP};
  assign memory[323] = {`NOP};
  assign memory[324] = {`NOP};
  assign memory[325] = {`NOP};
  assign memory[326] = {`NOP};
  assign memory[327] = {`NOP};
  assign memory[328] = {`NOP};
  assign memory[329] = {`NOP};
  assign memory[330] = {`NOP};
  assign memory[331] = {`NOP};
  assign memory[332] = {`NOP};
  assign memory[333] = {`NOP};
  assign memory[334] = {`NOP};
  assign memory[335] = {`NOP};
  assign memory[336] = {`NOP};
  assign memory[337] = {`NOP};
  assign memory[338] = {`NOP};
  assign memory[339] = {`NOP};
  assign memory[340] = {`NOP};
  assign memory[341] = {`NOP};
  assign memory[342] = {`NOP};
  assign memory[343] = {`NOP};
  assign memory[344] = {`NOP};
  assign memory[345] = {`NOP};
  assign memory[346] = {`NOP};
  assign memory[347] = {`NOP};
  assign memory[348] = {`NOP};
  assign memory[349] = {`NOP};
  assign memory[350] = {`NOP};
  assign memory[351] = {`NOP};
  assign memory[352] = {`NOP};
  assign memory[353] = {`NOP};
  assign memory[354] = {`NOP};
  assign memory[355] = {`NOP};
  assign memory[356] = {`NOP};
  assign memory[357] = {`NOP};
  assign memory[358] = {`NOP};
  assign memory[359] = {`NOP};
  assign memory[360] = {`NOP};
  assign memory[361] = {`NOP};
  assign memory[362] = {`NOP};
  assign memory[363] = {`NOP};
  assign memory[364] = {`NOP};
  assign memory[365] = {`NOP};
  assign memory[366] = {`NOP};
  assign memory[367] = {`NOP};
  assign memory[368] = {`NOP};
  assign memory[369] = {`NOP};
  assign memory[370] = {`NOP};
  assign memory[371] = {`NOP};
  assign memory[372] = {`NOP};
  assign memory[373] = {`NOP};
  assign memory[374] = {`NOP};
  assign memory[375] = {`NOP};
  assign memory[376] = {`NOP};
  assign memory[377] = {`NOP};
  assign memory[378] = {`NOP};
  assign memory[379] = {`NOP};
  assign memory[380] = {`NOP};
  assign memory[381] = {`NOP};
  assign memory[382] = {`NOP};
  assign memory[383] = {`NOP};
  assign memory[384] = {`NOP};
  assign memory[385] = {`NOP};
  assign memory[386] = {`NOP};
  assign memory[387] = {`NOP};
  assign memory[388] = {`NOP};
  assign memory[389] = {`NOP};
  assign memory[390] = {`NOP};
  assign memory[391] = {`NOP};
  assign memory[392] = {`NOP};
  assign memory[393] = {`NOP};
  assign memory[394] = {`NOP};
  assign memory[395] = {`NOP};
  assign memory[396] = {`NOP};
  assign memory[397] = {`NOP};
  assign memory[398] = {`NOP};
  assign memory[399] = {`NOP};
  assign memory[400] = {`NOP};
  assign memory[401] = {`NOP};
  assign memory[402] = {`NOP};
  assign memory[403] = {`NOP};
  assign memory[404] = {`NOP};
  assign memory[405] = {`NOP};
  assign memory[406] = {`NOP};
  assign memory[407] = {`NOP};
  assign memory[408] = {`NOP};
  assign memory[409] = {`NOP};
  assign memory[410] = {`NOP};
  assign memory[411] = {`NOP};
  assign memory[412] = {`NOP};
  assign memory[413] = {`NOP};
  assign memory[414] = {`NOP};
  assign memory[415] = {`NOP};
  assign memory[416] = {`NOP};
  assign memory[417] = {`NOP};
  assign memory[418] = {`NOP};
  assign memory[419] = {`NOP};
  assign memory[420] = {`NOP};
  assign memory[421] = {`NOP};
  assign memory[422] = {`NOP};
  assign memory[423] = {`NOP};
  assign memory[424] = {`NOP};
  assign memory[425] = {`NOP};
  assign memory[426] = {`NOP};
  assign memory[427] = {`NOP};
  assign memory[428] = {`NOP};
  assign memory[429] = {`NOP};
  assign memory[430] = {`NOP};
  assign memory[431] = {`NOP};
  assign memory[432] = {`NOP};
  assign memory[433] = {`NOP};
  assign memory[434] = {`NOP};
  assign memory[435] = {`NOP};
  assign memory[436] = {`NOP};
  assign memory[437] = {`NOP};
  assign memory[438] = {`NOP};
  assign memory[439] = {`NOP};
  assign memory[440] = {`NOP};
  assign memory[441] = {`NOP};
  assign memory[442] = {`NOP};
  assign memory[443] = {`NOP};
  assign memory[444] = {`NOP};
  assign memory[445] = {`NOP};
  assign memory[446] = {`NOP};
  assign memory[447] = {`NOP};
  assign memory[448] = {`NOP};
  assign memory[449] = {`NOP};
  assign memory[450] = {`NOP};
  assign memory[451] = {`NOP};
  assign memory[452] = {`NOP};
  assign memory[453] = {`NOP};
  assign memory[454] = {`NOP};
  assign memory[455] = {`NOP};
  assign memory[456] = {`NOP};
  assign memory[457] = {`NOP};
  assign memory[458] = {`NOP};
  assign memory[459] = {`NOP};
  assign memory[460] = {`NOP};
  assign memory[461] = {`NOP};
  assign memory[462] = {`NOP};
  assign memory[463] = {`NOP};
  assign memory[464] = {`NOP};
  assign memory[465] = {`NOP};
  assign memory[466] = {`NOP};
  assign memory[467] = {`NOP};
  assign memory[468] = {`NOP};
  assign memory[469] = {`NOP};
  assign memory[470] = {`NOP};
  assign memory[471] = {`NOP};
  assign memory[472] = {`NOP};
  assign memory[473] = {`NOP};
  assign memory[474] = {`NOP};
  assign memory[475] = {`NOP};
  assign memory[476] = {`NOP};
  assign memory[477] = {`NOP};
  assign memory[478] = {`NOP};
  assign memory[479] = {`NOP};
  assign memory[480] = {`NOP};
  assign memory[481] = {`NOP};
  assign memory[482] = {`NOP};
  assign memory[483] = {`NOP};
  assign memory[484] = {`NOP};
  assign memory[485] = {`NOP};
  assign memory[486] = {`NOP};
  assign memory[487] = {`NOP};
  assign memory[488] = {`NOP};
  assign memory[489] = {`NOP};
  assign memory[490] = {`NOP};
  assign memory[491] = {`NOP};
  assign memory[492] = {`NOP};
  assign memory[493] = {`NOP};
  assign memory[494] = {`NOP};
  assign memory[495] = {`NOP};
  assign memory[496] = {`NOP};
  assign memory[497] = {`NOP};
  assign memory[498] = {`NOP};
  assign memory[499] = {`NOP};
  assign memory[500] = {`NOP};
  assign memory[501] = {`NOP};
  assign memory[502] = {`NOP};
  assign memory[503] = {`NOP};
  assign memory[504] = {`NOP};
  assign memory[505] = {`NOP};
  assign memory[506] = {`NOP};
  assign memory[507] = {`NOP};
  assign memory[508] = {`NOP};
  assign memory[509] = {`NOP};
  assign memory[510] = {`NOP};
  assign memory[511] = {`NOP};

endmodule
