/*
 *       Main: TMS320F28335
 *             (c) Alex Reis
 *
 *###########################################################################
 *
 * FILE:    Main.c
 *
 * TITLE:   DSP28335ControlCARD;
 *          Sistema de controle para inversor
 *
 *#################################################################################
 *  Ver | dd mmm yyyy | Who  | Description of changes
 * =====|=============|======|=====================================================
 *  0.1 | 01 Out 2016 | A.R. | Verão inicial do algoritmo: Testes das placas de
 *      |             |      | aquisição de sinal
 *---------------------------------------------------------------------------------
 *#################################################################################
*/

#include "DSP2833x_Device.h"

#include "DSP28x_Project.h"

// If not installed, install the "Control Suite".
// Navigate to "C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\include" and include the file "DSP28x_Project.h".

#include "Math.h"

// Diretivas de definição
#define ADC_CKPS 0x01
#define AMOSTRAS 16                      // Numero de amostras de tensão armazenadas internamente               //// -----> NO NOSSO CASO, SERÃO TAMBÉM 16? DOS DOIS LADOS?
#define PERI_PWM 1000000    //0.0004    // Periodo de chaveamento do inversor, considerando uma frequencia de 2500 Hz (0,4 ms) //// -----> NO ATP ESTAMOS UTILIZANDO 3500 Hz.
#define PERI_AMOSTRAGEM 0.0001          // Periodo de amostragem do controle, considerando uma frequencia de 10 kHz (0,1 ms)   //// -----> CONSIDERA-SE A MESMA FREQUÊNCIA?

// Estrutura para armazenamento das caracteristicas do sinal: amplitude e frequência
//struct DadosEstimados{
//    float Amplitude;        // Variável para armazenamento do valor de pico
//    float Frequencia;   // Variável para armazenamento da frequência estimada
//    float Zero;         // Variável para armazenamento da última por zero
//};


// Funções preexistentes nas bibliotecas da TI para configuração da CPU                                                                //// -----> SERÃO AS MESMAS FUNÇÕES?
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void InitAdc(void);
//extern void ConfigCpuTimer(struct CPUTIMER_VARS*, float, float);


// Prototype statements for functions found within this file.                                                                         //// -----> AQUI PROVAVELMENTE CRIAREMOS UMA "void smo" EM RET.
//-----------------------------------------------------------
void Gpio_select(void);                             // Configuração das saídas digitais
interrupt void cpu_timer0_isr(void);                // Função associada à interrupção vinculada ao Timer 0
interrupt void adc_isr(void);                       // Função associada à interrupção vinculada ao ADC
//void SpaceVectorPWM(void);                          // Função para determinação dos pulsos PWM
void DSOGI_FLL(float Va, float Vb, float Vc);       // Função para o DSOGI-FLL
//struct DadosEstimados Data_Estimation(float *Voltage);
void Data_Estimation(void); // Função que determina o valor máximo de tensão e correntes lidos
void Setup_ePWM(void); // Função que configura os canais ePWM1, ePWM2 e ePWM3 para gerar os pulsos de chaveamento
void Change_ePWM(float InvFaseA, float InvFaseB, float InvFaseC); // Função que altera o duty cycle dos pulsos de chaveamento conforme as ondas modulantes

// Variaveis globais utilizadas no programa
// #############################################################################

// Variaveis para armazenamento das caracteristicas do sinal
float V_A = 0; // Tensao máxima obtida
float I_A = 0, I_B = 0, I_C = 0;
float I_Af[2] = {0}, I_Bf[2] = {0}, I_Cf[2] = {0}; // Corrente máxima obtida

// Váriáveis associadas à placa de aquisição de sinais
// ---------------------------------------------------

const float PI = 3.1415926535;  // Definição do valor de pi
const float V_base = 380.0;     // Valor da tensão de linha utilizada como base para os cálculos
const float S_base = 10000; //5000;        // Valor da potência base (trifásica)                                                                 //// -----> UTILIZAR O MESMO VALOR DE POTÊNCIA DE BASE DO ATP
const float I_base = 15.193428;
const float Z_base = 14.44;

/*
 * KV_PCI: Ganho da placa de condicionamento de sinal, considerando as alterações provocadas pelo transdutor de
 * tensão e pelo filtro anti-aliasing. A utilização deste ganho permite transformar o sinal de saída da
 * PCI (Vsaida) no sinal de entrada (Ventrada ou Ientrada). As transformações seguem as seguintes proporções:
 * 1. Transdutor de tensão: para uma entrada de 220 V, tem-se 2,5 V na sua saída
 * 2. Transdutor de corrente: para uma entrada de 50 A, tem-se 5,0 V na sua saída
 * 3. Filtro anti-aliasing: para evitar a saturação do sinal, o filtro atenua os sinais em 0.303030303030
 *
 * Logo:            2.5 * 0,3030303030 * Ventrada                               220
 *        Vsaida =  -----------------------------       ->  KV_PCI = -------------------- = 167,64
 *                              220                         0,3030303030 * 2,5
 *                  5,0 * 0,3030303030 * Ientrada                              50
 *        Vsaida =  -----------------------------       ->  KI_PCI = -------------------- = 33,0
 *                              50                          0,3030303030 * 5,0
 */
const float KV_PCI = 261.36;//290.40; //13.2; // 290.40; // 167.64;
const float KI_PCI = 33.0/4.0;    // AINDA DEVE SER AJUSTADO CORRETAMENTE                                       //// -----> NÃO ENTENDI COMO AJUSTAR ISTO
/*
 * Constante utilizada para transformar o valor de saida do conversor A/D no valor de entrada.  A mesma segue
 * a resolução do conversor A/D do DSP (3,3 V; 12 bits - 4095)
 * Kad = 3,3/4095 = 0.000805860806                                                            //// -----> OK, OS VALORES SÃO OS MESMOS?
 */
const float Kad = 0.000805860806;

// Variáveis para armazenamento da saída do conversor AD
// -----------------------------------------------------
int AD_output1 = 0, AD_output2 = 0, AD_output3 = 0, AD_output4 = 0, AD_output5 = 0, AD_output6 = 0, AD_output7 = 0;           //// -----> OK, CONTINUAREMOS COM 7 SAÍDAS? E DO LADO DO RET.?

// Vetores para armazenamento dos dados amostrados - valore reais                                     //// -----> OK PARA INV., MAS E RET.?
// --------------------------------------------------------------
float Corrente_A[AMOSTRAS]  = {0};
float Corrente_B[AMOSTRAS]  = {0};
float Corrente_C[AMOSTRAS]  = {0};
float Tensao_A[AMOSTRAS]    = {0};
float Tensao_B[AMOSTRAS]    = {0};
float Tensao_C[AMOSTRAS]    = {0};
float Vcc[AMOSTRAS]     = {0};


// Variaveis para as tensoes de entrada filtradas

float Tensao_Af[2] = {0,0};
float Tensao_Bf[2] = {0,0};
float Tensao_Cf[2] = {0,0};


// Variaveis para implementação do DSOGI-FLL                                                     //// -----> OK PARA INV., PARA RET. SERÃO VARIÁVEIS DO SMO.
const float k = 1.41424356, Tau = 266.579;// 50.0;
float V_alfa_inter[2] = {0}, V_alfa_out[2] = {0}, V_alfa_int[2] = {0};
float V_beta_inter[2] = {0}, V_beta_out[2] = {0}, V_beta_int[2] = {0};
float V_alfa_P = 0, V_alfa_N = 0, V_beta_P = 0, V_beta_N = 0;
float FLL_int[2] = {0}, FLL_Freq[2] = {0}, Freq_in = 0, Freq_Rede = 376.99111841;
float Teta = 0;
unsigned int first_time = 0;


// Variáveis para implementação do controle vetorial (sistema síncrono (DQ0) com as tensões no PAC)
float Vd_med[2] = {0, 0}, Vq_med[2] = {0, 0}, Vd_medf[2] = {0, 0}, Vq_medf[2] = {0, 0};
float Id_med[2] = {0, 0}, Iq_med[2] = {0, 0}, Id_medf[2] = {0, 0}, Iq_medf[2] = {0, 0};

// Indutância do filtro de saída, em H
const float L_Filt = 0.003; // O filtro que está sendo utilizado é de 3 mH

// Indutância do filtro de saída, em pu, para cálculo dos termos de acoplamento entre malhas de controle
const float L_pu = 0.039461127260815; //+ 0.0003; // O valor 0.0003 refere-se a impedância do transformador em pu

// Valores de tensão no capacitor

float V_CAP[2] = {0,0}, V_CAPF[2] = {0,0};

// Valores em pu das tensões medidas

float Vd_pu = 0, Vq_pu = 0;

// Valores em pu das correntes medidas

float Id_pu = 0, Iq_pu = 0;

// Valores das potências trifásicas no sistemas dq0, em pu (utilizando dados de nova transformação vetorial)
float P_med[2] = {0,0}, Q_med[2] = {0,0};
float P_medf[2] = {0,0}, Q_medf[2] = {0,0};

// Valores de erro de potência
float Erro_P[2] = {0,0}, Erro_Q[2] = {0,0};
float Id_ref[2] = {0, 0}, Iq_ref[2] = {0, 0};

// Ganhos dos controladores PI de Potência Ativa
const float K1 = 0.7523, K2 = 0.7477;

// Ganhos dos controladores PI de Potência Reativa
const float K3 = 0.739, K4 = 0.7269;

// Ganhos dos controladores PI de Corrente
const float K5 = 0.7358, K6 = 0.7242;

// Valores de referência para os controladores de corrente de eixo direto e em quadratura
//float Id_ref[2] = {0, 0}, Iq_ref[2] = {0, 0};

// Erro entre valores de referência e medidos, para correntes de eixo direto e em quadratura
float Erro_id[2] = {0, 0}, Erro_iq[2] = {0, 0};

// Ganhos dos controladores PI da malha de corrente de eixo direto e em quadratura
//float Kp_CC = 1, Ki_CC = 1;

// Valores de saída dos controladores PI das malhas de corrente de eixo direto e em quadratura
float Pi_id[2] = {0, 0}, Pi_iq[2] = {0, 0};

// Valores das tensões a serem sintetizadas nos terminais do inversor
float Ed_inv = 0, Eq_inv = 0, InvFaseA = 0, InvFaseB = 0, InvFaseC = 0;

int Chaves[6] = {1, 1, 1, 0, 0, 0}; // Matriz para armazenamento das posições das chaves.                           //// -----> NÃO ENTENDI ESSAS CHAVES.
                       // A mesma é utilizada para alterar o estado das saidas digitais.

int Status = 0;                     // Habilita o novo cálculo do estado do inversor

// Fim das variaveis globais utilizadas no programa
// #############################################################################


// Variaveis e funcao para verificacao da PLL

float Vpll = 0;
void teste_PLL(void);


// Vetores para teste da FLL                                                        //// -----> E ESSES VETORES? DE ONDE VÊM?
//int FaseA[834] = {1329,1329,1326,1321,1314,1306,1296,1283,1269,1254,1236,1217,1196,1173,1149,1122,1095,1066,1035,1003,969,934,898,860,821,781,740,698,655,611,566,520,474,427,379,331,282,233,183,133,83,33,-17,-67,-117,-167,-216,-266,-314,-363,-411,-458,-505,-551,-596,-640,-684,-726,-768,-808,-847,-885,-922,-958,-992,-1024,-1056,-1085,-1113,-1140,-1165,-1188,-1210,-1230,-1248,-1264,-1279,-1292,-1303,-1312,-1319,-1324,-1328,-1329,-1329,-1327,-1323,-1317,-1309,-1299,-1288,-1274,-1259,-1242,-1223,-1203,-1181,-1157,-1131,-1104,-1076,-1045,-1014,-980,-946,-910,-873,-834,-795,-754,-712,-670,-626,-581,-536,-489,-442,-395,-347,-298,-249,-200,-150,-100,-50,-0,50,100,150,200,249,298,347,395,442,489,536,581,626,670,712,754,795,834,873,910,946,980,1014,1045,1076,1104,1131,1157,1181,1203,1223,1242,1259,1274,1288,1299,1309,1317,1323,1327,1329,1329,1328,1324,1319,1312,1303,1292,1279,1264,1248,1230,1210,1188,1165,1140,1113,1085,1056,1024,992,958,922,885,847,808,768,726,684,640,596,551,505,458,411,363,314,266,216,167,117,67,17,-33,-83,-133,-183,-233,-282,-331,-379,-427,-474,-520,-566,-611,-655,-698,-740,-781,-821,-860,-898,-934,-969,-1003,-1035,-1066,-1095,-1122,-1149,-1173,-1196,-1217,-1236,-1254,-1269,-1283,-1296,-1306,-1314,-1321,-1326,-1329,-1329,-1329,-1326,-1321,-1314,-1306,-1296,-1283,-1269,-1254,-1236,-1217,-1196,-1173,-1149,-1122,-1095,-1066,-1035,-1003,-969,-934,-898,-860,-821,-781,-740,-698,-655,-611,-566,-520,-474,-427,-379,-331,-282,-233,-183,-133,-83,-33,17,67,117,167,216,266,314,363,411,458,505,551,596,640,684,726,768,808,847,885,922,958,992,1024,1056,1085,1113,1140,1165,1188,1210,1230,1248,1264,1279,1292,1303,1312,1319,1324,1328,1329,1329,1327,1323,1317,1309,1299,1288,1274,1259,1242,1223,1203,1181,1157,1131,1104,1076,1045,1014,980,946,910,873,834,795,754,712,670,626,581,536,489,442,395,347,298,249,200,150,100,50,-0,-50,-100,-150,-200,-249,-298,-347,-395,-442,-489,-536,-581,-626,-670,-712,-754,-795,-834,-873,-910,-946,-980,-1014,-1045,-1076,-1104,-1131,-1157,-1181,-1203,-1223,-1242,-1259,-1274,-1288,-1299,-1309,-1317,-1323,-1327,-1329,-1329,-1328,-1324,-1319,-1312,-1303,-1292,-1279,-1264,-1248,-1230,-1210,-1188,-1165,-1140,-1113,-1085,-1056,-1024,-992,-958,-922,-885,-847,-808,-768,-726,-684,-640,-596,-551,-505,-458,-411,-363,-314,-266,-216,-167,-117,-67,-17,33,83,133,183,233,282,331,379,427,474,520,566,611,655,698,740,781,821,860,898,934,969,1003,1035,1066,1095,1122,1149,1173,1196,1217,1236,1254,1269,1283,1296,1306,1314,1321,1326,1329,1329,1329,1326,1321,1314,1306,1296,1283,1269,1254,1236,1217,1196,1173,1149,1122,1095,1066,1035,1003,969,934,898,860,821,781,740,698,655,611,566,520,474,427,379,331,282,233,183,133,83,33,-17,-67,-117,-167,-216,-266,-314,-363,-411,-458,-505,-551,-596,-640,-684,-726,-768,-808,-847,-885,-922,-958,-992,-1024,-1056,-1085,-1113,-1140,-1165,-1188,-1210,-1230,-1248,-1264,-1279,-1292,-1303,-1312,-1319,-1324,-1328,-1329,-1329,-1327,-1323,-1317,-1309,-1299,-1288,-1274,-1259,-1242,-1223,-1203,-1181,-1157,-1131,-1104,-1076,-1045,-1014,-980,-946,-910,-873,-834,-795,-754,-712,-670,-626,-581,-536,-489,-442,-395,-347,-298,-249,-200,-150,-100,-50,0,50,100,150,200,249,298,347,395,442,489,536,581,626,670,712,754,795,834,873,910,946,980,1014,1045,1076,1104,1131,1157,1181,1203,1223,1242,1259,1274,1288,1299,1309,1317,1323,1327,1329,1329,1328,1324,1319,1312,1303,1292,1279,1264,1248,1230,1210,1188,1165,1140,1113,1085,1056,1024,992,958,922,885,847,808,768,726,684,640,596,551,505,458,411,363,314,266,216,167,117,67,17,-33,-83,-133,-183,-233,-282,-331,-379,-427,-474,-520,-566,-611,-655,-698,-740,-781,-821,-860,-898,-934,-969,-1003,-1035,-1066,-1095,-1122,-1149,-1173,-1196,-1217,-1236,-1254,-1269,-1283,-1296,-1306,-1314,-1321,-1326,-1329,-1329,-1329,-1326,-1321,-1314,-1306,-1296,-1283,-1269,-1254,-1236,-1217,-1196,-1173,-1149,-1122,-1095,-1066,-1035,-1003,-969,-934,-898,-860,-821,-781,-740,-698,-655,-611,-566,-520,-474,-427,-379,-331,-282,-233,-183,-133,-83,-33,17,67,117,167,216,266,314,363,411,458,505,551,596,640,684,726,768,808,847,885,922,958,992,1024,1056,1085,1113,1140,1165,1188,1210,1230,1248,1264,1279,1292,1303,1312,1319,1324,1328,1329};
//int FaseB[834] = {-665,-621,-576,-531,-484,-437,-390,-341,-293,-244,-194,-145,-95,-45,6,56,106,156,205,255,304,352,400,448,495,541,586,631,674,717,759,799,839,877,914,950,984,1017,1049,1079,1107,1134,1160,1183,1205,1226,1244,1261,1276,1289,1300,1310,1318,1323,1327,1329,1329,1327,1324,1318,1311,1302,1290,1277,1263,1246,1228,1208,1186,1162,1137,1110,1082,1052,1021,988,954,918,881,843,804,763,722,679,636,591,546,500,453,406,358,309,260,211,161,111,61,11,-39,-89,-139,-189,-238,-287,-336,-384,-432,-479,-525,-571,-616,-660,-703,-745,-786,-826,-864,-902,-938,-973,-1006,-1038,-1069,-1098,-1125,-1151,-1176,-1198,-1219,-1238,-1255,-1271,-1285,-1297,-1307,-1315,-1322,-1326,-1329,-1329,-1328,-1325,-1320,-1314,-1305,-1294,-1282,-1268,-1252,-1234,-1215,-1193,-1170,-1146,-1119,-1092,-1062,-1031,-999,-965,-930,-894,-856,-817,-777,-736,-693,-650,-606,-561,-515,-469,-421,-374,-325,-276,-227,-178,-128,-78,-28,22,72,122,172,222,271,320,368,416,463,510,556,601,645,689,731,772,813,852,890,926,961,995,1028,1059,1088,1116,1143,1168,1191,1212,1232,1250,1266,1280,1293,1304,1313,1320,1325,1328,1329,1329,1326,1322,1316,1308,1298,1286,1273,1257,1240,1221,1201,1178,1154,1128,1101,1072,1042,1010,977,942,906,869,830,790,750,708,665,621,576,531,484,437,390,341,293,244,194,145,95,45,-6,-56,-106,-156,-205,-255,-304,-352,-400,-448,-495,-541,-586,-631,-674,-717,-759,-799,-839,-877,-914,-950,-984,-1017,-1049,-1079,-1107,-1134,-1160,-1183,-1205,-1226,-1244,-1261,-1276,-1289,-1300,-1310,-1318,-1323,-1327,-1329,-1329,-1327,-1324,-1318,-1311,-1302,-1290,-1277,-1263,-1246,-1228,-1208,-1186,-1162,-1137,-1110,-1082,-1052,-1021,-988,-954,-918,-881,-843,-804,-763,-722,-679,-636,-591,-546,-500,-453,-406,-358,-309,-260,-211,-161,-111,-61,-11,39,89,139,189,238,287,336,384,432,479,525,571,616,660,703,745,786,826,864,902,938,973,1006,1038,1069,1098,1125,1151,1176,1198,1219,1238,1255,1271,1285,1297,1307,1315,1322,1326,1329,1329,1328,1325,1320,1314,1305,1294,1282,1268,1252,1234,1215,1193,1170,1146,1119,1092,1062,1031,999,965,930,894,856,817,777,736,693,650,606,561,515,469,421,374,325,276,227,178,128,78,28,-22,-72,-122,-172,-222,-271,-320,-368,-416,-463,-510,-556,-601,-645,-689,-731,-772,-813,-852,-890,-926,-961,-995,-1028,-1059,-1088,-1116,-1143,-1168,-1191,-1212,-1232,-1250,-1266,-1280,-1293,-1304,-1313,-1320,-1325,-1328,-1329,-1329,-1326,-1322,-1316,-1308,-1298,-1286,-1273,-1257,-1240,-1221,-1201,-1178,-1154,-1128,-1101,-1072,-1042,-1010,-977,-942,-906,-869,-830,-790,-750,-708,-665,-621,-576,-531,-484,-437,-390,-341,-293,-244,-194,-145,-95,-45,6,56,106,156,205,255,304,352,400,448,495,541,586,631,674,717,759,799,839,877,914,950,984,1017,1049,1079,1107,1134,1160,1183,1205,1226,1244,1261,1276,1289,1300,1310,1318,1323,1327,1329,1329,1327,1324,1318,1311,1302,1290,1277,1263,1246,1228,1208,1186,1162,1137,1110,1082,1052,1021,988,954,918,881,843,804,763,722,679,636,591,546,500,453,406,358,309,260,211,161,111,61,11,-39,-89,-139,-189,-238,-287,-336,-384,-432,-479,-525,-571,-616,-660,-703,-745,-786,-826,-864,-902,-938,-973,-1006,-1038,-1069,-1098,-1125,-1151,-1176,-1198,-1219,-1238,-1255,-1271,-1285,-1297,-1307,-1315,-1322,-1326,-1329,-1329,-1328,-1325,-1320,-1314,-1305,-1294,-1282,-1268,-1252,-1234,-1215,-1193,-1170,-1146,-1119,-1092,-1062,-1031,-999,-965,-930,-894,-856,-817,-777,-736,-693,-650,-606,-561,-515,-469,-421,-374,-325,-276,-227,-178,-128,-78,-28,22,72,122,172,222,271,320,368,416,463,510,556,601,645,689,731,772,813,852,890,926,961,995,1028,1059,1088,1116,1143,1168,1191,1212,1232,1250,1266,1280,1293,1304,1313,1320,1325,1328,1329,1329,1326,1322,1316,1308,1298,1286,1273,1257,1240,1221,1201,1178,1154,1128,1101,1072,1042,1010,977,942,906,869,830,790,750,708,665,621,576,531,484,437,390,341,293,244,194,145,95,45,-6,-56,-106,-156,-205,-255,-304,-352,-400,-448,-495,-541,-586,-631,-674,-717,-759,-799,-839,-877,-914,-950,-984,-1017,-1049,-1079,-1107,-1134,-1160,-1183,-1205,-1226,-1244,-1261,-1276,-1289,-1300,-1310,-1318,-1323,-1327,-1329,-1329,-1327,-1324,-1318,-1311,-1302,-1290,-1277,-1263,-1246,-1228,-1208,-1186,-1162,-1137,-1110,-1082,-1052,-1021,-988,-954,-918,-881,-843,-804,-763,-722,-679};
//int FaseC[834] = {-665,-708,-750,-790,-830,-869,-906,-942,-977,-1010,-1042,-1072,-1101,-1128,-1154,-1178,-1201,-1221,-1240,-1257,-1273,-1286,-1298,-1308,-1316,-1322,-1326,-1329,-1329,-1328,-1325,-1320,-1313,-1304,-1293,-1280,-1266,-1250,-1232,-1212,-1191,-1168,-1143,-1116,-1088,-1059,-1028,-995,-961,-926,-890,-852,-813,-772,-731,-689,-645,-601,-556,-510,-463,-416,-368,-320,-271,-222,-172,-122,-72,-22,28,78,128,178,227,276,325,374,421,469,515,561,606,650,693,736,777,817,856,894,930,965,999,1031,1062,1092,1119,1146,1170,1193,1215,1234,1252,1268,1282,1294,1305,1314,1320,1325,1328,1329,1329,1326,1322,1315,1307,1297,1285,1271,1255,1238,1219,1198,1176,1151,1125,1098,1069,1038,1006,973,938,902,864,826,786,745,703,660,616,571,525,479,432,384,336,287,238,189,139,89,39,-11,-61,-111,-161,-211,-260,-309,-358,-406,-453,-500,-546,-591,-636,-679,-722,-763,-804,-843,-881,-918,-954,-988,-1021,-1052,-1082,-1110,-1137,-1162,-1186,-1208,-1228,-1246,-1263,-1277,-1290,-1302,-1311,-1318,-1324,-1327,-1329,-1329,-1327,-1323,-1318,-1310,-1300,-1289,-1276,-1261,-1244,-1226,-1205,-1183,-1160,-1134,-1107,-1079,-1049,-1017,-984,-950,-914,-877,-839,-799,-759,-717,-674,-631,-586,-541,-495,-448,-400,-352,-304,-255,-205,-156,-106,-56,-6,45,95,145,194,244,293,341,390,437,484,531,576,621,665,708,750,790,830,869,906,942,977,1010,1042,1072,1101,1128,1154,1178,1201,1221,1240,1257,1273,1286,1298,1308,1316,1322,1326,1329,1329,1328,1325,1320,1313,1304,1293,1280,1266,1250,1232,1212,1191,1168,1143,1116,1088,1059,1028,995,961,926,890,852,813,772,731,689,645,601,556,510,463,416,368,320,271,222,172,122,72,22,-28,-78,-128,-178,-227,-276,-325,-374,-421,-469,-515,-561,-606,-650,-693,-736,-777,-817,-856,-894,-930,-965,-999,-1031,-1062,-1092,-1119,-1146,-1170,-1193,-1215,-1234,-1252,-1268,-1282,-1294,-1305,-1314,-1320,-1325,-1328,-1329,-1329,-1326,-1322,-1315,-1307,-1297,-1285,-1271,-1255,-1238,-1219,-1198,-1176,-1151,-1125,-1098,-1069,-1038,-1006,-973,-938,-902,-864,-826,-786,-745,-703,-660,-616,-571,-525,-479,-432,-384,-336,-287,-238,-189,-139,-89,-39,11,61,111,161,211,260,309,358,406,453,500,546,591,636,679,722,763,804,843,881,918,954,988,1021,1052,1082,1110,1137,1162,1186,1208,1228,1246,1263,1277,1290,1302,1311,1318,1324,1327,1329,1329,1327,1323,1318,1310,1300,1289,1276,1261,1244,1226,1205,1183,1160,1134,1107,1079,1049,1017,984,950,914,877,839,799,759,717,674,631,586,541,495,448,400,352,304,255,205,156,106,56,6,-45,-95,-145,-194,-244,-293,-341,-390,-437,-484,-531,-576,-621,-665,-708,-750,-790,-830,-869,-906,-942,-977,-1010,-1042,-1072,-1101,-1128,-1154,-1178,-1201,-1221,-1240,-1257,-1273,-1286,-1298,-1308,-1316,-1322,-1326,-1329,-1329,-1328,-1325,-1320,-1313,-1304,-1293,-1280,-1266,-1250,-1232,-1212,-1191,-1168,-1143,-1116,-1088,-1059,-1028,-995,-961,-926,-890,-852,-813,-772,-731,-689,-645,-601,-556,-510,-463,-416,-368,-320,-271,-222,-172,-122,-72,-22,28,78,128,178,227,276,325,374,421,469,515,561,606,650,693,736,777,817,856,894,930,965,999,1031,1062,1092,1119,1146,1170,1193,1215,1234,1252,1268,1282,1294,1305,1314,1320,1325,1328,1329,1329,1326,1322,1315,1307,1297,1285,1271,1255,1238,1219,1198,1176,1151,1125,1098,1069,1038,1006,973,938,902,864,826,786,745,703,660,616,571,525,479,432,384,336,287,238,189,139,89,39,-11,-61,-111,-161,-211,-260,-309,-358,-406,-453,-500,-546,-591,-636,-679,-722,-763,-804,-843,-881,-918,-954,-988,-1021,-1052,-1082,-1110,-1137,-1162,-1186,-1208,-1228,-1246,-1263,-1277,-1290,-1302,-1311,-1318,-1324,-1327,-1329,-1329,-1327,-1323,-1318,-1310,-1300,-1289,-1276,-1261,-1244,-1226,-1205,-1183,-1160,-1134,-1107,-1079,-1049,-1017,-984,-950,-914,-877,-839,-799,-759,-717,-674,-631,-586,-541,-495,-448,-400,-352,-304,-255,-205,-156,-106,-56,-6,45,95,145,194,244,293,341,390,437,484,531,576,621,665,708,750,790,830,869,906,942,977,1010,1042,1072,1101,1128,1154,1178,1201,1221,1240,1257,1273,1286,1298,1308,1316,1322,1326,1329,1329,1328,1325,1320,1313,1304,1293,1280,1266,1250,1232,1212,1191,1168,1143,1116,1088,1059,1028,995,961,926,890,852,813,772,731,689,645,601,556,510,463,416,368,320,271,222,172,122,72,22,-28,-78,-128,-178,-227,-276,-325,-374,-421,-469,-515,-561,-606,-650};

float time_pwm = 0.0; // variável para teste do pwm

//###########################################################################
//                      main code                                           //// -----> DIFÍCIL ENTENDER, TIRANDO DSOGI-PLL E DQ0
//###########################################################################
void main(void)
{
    // Basic Core Initialization
    InitSysCtrl();

    EALLOW;
    #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
        #define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
    #endif
    #if (CPU_FRQ_100MHZ)
        #define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
    #endif
    EDIS;

    // Define ADCCLK clock frequency ( less than or equal to 25 MHz )
    // Assuming InitSysCtrl() has set SYSCLKOUT to 150 MHz
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

    //WD activation
    EALLOW;
    SysCtrlRegs.WDCR = 0x006F;
    SysCtrlRegs.SCSR = 0x00;
    EDIS;

    // Disable all interrupts
    DINT;

    // GPIO Configuration
    // ------------------
    Gpio_select();

    // -------------------

    // Interruption service initialization
    InitPieCtrl();
    InitPieVectTable();


    // ADC Initialization
    // #######################################

    // Inicialização básica do conversor A/D
    InitAdc();

    //AdcRegs.ADCTRL1.all = 0;

    //Sequencer mode: cascade mode
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;

    //Autosequencer para no final da sequencia de amostragem
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;

    //Tempo de aquisição
    //AdcRegs.ADCTRL1.bit.ACQ_PS = 3;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 5;

    //Janela de aquisição
    AdcRegs.ADCTRL1.bit.CPS = 0;

    // Definição do clock FCLK em 12.5 MHz
    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;

    // Definição do numero de canais que o conversor A/D deve ler (n+1)
    AdcRegs.ADCMAXCONV.all = 0x6;

    //Canais de leitura
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;  //ADCINA0 -> Fase A - Tensão
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;  //ADCINA1 -> Fase B - Tensão ---> A PLL está em fase com este canal do A/D
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;  //ADCINA2 -> Fase C - Tensão
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;  //ADCINA3 -> Fase A - Corrente
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;  //ADCINA4 -> Fase B - Corrente
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;  //ADCINA5 -> Fase C - Corrente
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;  //ADCINA6 -> Vcc - Tensão

    // Conversor A/D é ativado por pulso PWM
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    // Ativação da interrupção associada ao termino da amostragem
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
    // Interrupções são executadas a cada final de sequencia do Autosequencer
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;

    // ##############################################

    // ePWM2 initialization
    // ##################################
    EPwm4Regs.TBCTL.all = 0xC030;       // Configure timer control register
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;     // CLKDIV = 1
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 1;  // HSPCLKDIV = 2
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;    // Contagem up
    EPwm4Regs.TBPRD = 7500;         // Frequencia do PWM = 10 kHz
    EPwm4Regs.ETPS.all = 0x0100;        // Configure ADC start by ePWM2
    EPwm4Regs.ETSEL.all = 0x0A00;       // Enable SOCA to ADC
    //#########################################

    // ########################################
    // Initialization of ePWM1, ePWM2 and ePWM3

    Setup_ePWM();

    // ########################################

    // Configuração das interrupções
    // #############################

    EALLOW;

    // Ativação da interrupção associada ao conversor A/D
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    // Associação da função "adc_isr" à interrupção do conversor A/D
    PieVectTable.ADCINT = &adc_isr;

    // Ativação da interrupção associada  ao Timer 0
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Associação da função "cpu_timer0_isr" à interrupção do Timer0
    //PieVectTable.TINT0 = &cpu_timer0_isr;

    EDIS;

    // Inicialização dos timers
    //InitCpuTimers();

    // Configuração inicial do Timer 0: o mesmo é utilizado para a contagem do periodo de chaveamento
    //ConfigCpuTimer(&CpuTimer0, 150, PERI_PWM);

    // Ativação de interrupções
    IER |= 1;
    EINT;
    ERTM;

    //Início da contagem do tempo - Timer 0
    //CpuTimer0Regs.TCR.bit.TSS = 0;

    //#########################################

    while(1)
    {
        EALLOW;
        SysCtrlRegs.WDKEY = 0x55;
        SysCtrlRegs.WDKEY = 0xAA;
        EDIS;
    }

}

//###########################################################################
//                      End of main code
//###########################################################################

void Gpio_select(void)
{
    EALLOW;
    // Set all Mux to use GPIO
    GpioCtrlRegs.GPAMUX1.all = 0;   // GPIO15 ... GPIO0 = General Puropse I/O
    GpioCtrlRegs.GPAMUX2.all = 0;   // GPIO31 ... GPIO16 = General Purpose I/O
    GpioCtrlRegs.GPBMUX1.all = 0;   // GPIO47 ... GPIO32 = General Purpose I/O
    GpioCtrlRegs.GPBMUX2.all = 0;   // GPIO63 ... GPIO48 = General Purpose I/O
    GpioCtrlRegs.GPCMUX1.all = 0;   // GPIO79 ... GPIO64 = General Purpose I/O
    GpioCtrlRegs.GPCMUX2.all = 0;   // GPIO87 ... GPIO80 = General Purpose I/O

    // All GPIO are defined as input ports
    GpioCtrlRegs.GPADIR.all = 0;
    GpioCtrlRegs.GPBDIR.all = 0;
    GpioCtrlRegs.GPCDIR.all = 0;

    // peripheral explorer: LED LD3 at GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

    // GPIO33 = GPIO para recebimento do status do disjuntor
    //GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;

    // GPIO de envio de sinal de fechamento das chaves superiores
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;     // GPIO32 = Fase A
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;     // GPIO30 = Fase B
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;     // GPIO28 = Fase C

    // GPIO de envio de sinal de fechamento das chaves inferiores
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // GPIO32 = Fase A
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;     // GPIO30 = Fase B
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;     // GPIO28 = Fase C

    // GPIO para verificação do status da PLL
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // GPIO10 = Teste da PLL

    EDIS;

    //Inicialização do estado das chaves - Todas estão inicialmente abertas
    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;

    //Inicialização da GPIO da PLL - Pino inicialmente em 0

    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;

}

interrupt void cpu_timer0_isr(void)
{
    /* Esta função é utlizada para atualizar o estado das saidas digitais, as
     * quais se destinam a comandar as chaves do inversor.
     */

    //Contador de execução da interrupção
    CpuTimer0.InterruptCount++;

    // Acendimento do LED: indicativo de passagem pelo código
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // Habilita o novo calculo do estado do inversor
    Status = 0;

    // Atualização do estado das chaves
    if (Chaves[0] == 1){
        GpioDataRegs.GPBSET.bit.GPIO32 = 1; // Fase A - Chave superior
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // Fase A - Chave inferior
    }
    else{
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // Fase A - Chave superior
        GpioDataRegs.GPASET.bit.GPIO31 = 1; // Fase A - Chave inferior
    }

    if (Chaves[1] == 1){
        GpioDataRegs.GPASET.bit.GPIO30 = 1; // Fase B - Chave superior
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;   // Fase B - Chave inferior
    }
    else{
        GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;   // Fase B - Chave superior
        GpioDataRegs.GPASET.bit.GPIO29 = 1; // Fase B - Chave inferior
    }

    if (Chaves[2] == 1){
        GpioDataRegs.GPASET.bit.GPIO28 = 1;     // Fase C - Chave superior
        GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;   // Fase C - Chave inferior
    }
    else{
        GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;   // Fase C - Chave superior
        GpioDataRegs.GPASET.bit.GPIO23 = 1; // Fase C - Chave inferior
    }


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    EALLOW;
    SysCtrlRegs.WDKEY = 0x55;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
}

interrupt void adc_isr(void)
{
    int jj;     // Variável temporária para contagem

    //Deslocamento dos valores do array para a esquerda
    for (jj=0;jj<(AMOSTRAS-1);jj++){
        Corrente_A[jj] = Corrente_A[jj+1];
        Corrente_B[jj] = Corrente_B[jj+1];
        Corrente_C[jj] = Corrente_C[jj+1];
        Tensao_A[jj] = Tensao_A[jj+1];
        Tensao_B[jj] = Tensao_B[jj+1];
        Tensao_C[jj] = Tensao_C[jj+1];
        Vcc[jj] = Vcc[jj + 1];
    }

    // Amostragem das tensões e correntes. O offset está de acordo com
    // o ajuste da placa de aquisição de dados
    AD_output1 = (AdcRegs.ADCRESULT0>>4) - 1862;
    AD_output2 = (AdcMirror.ADCRESULT1) - 1860;
    AD_output3 = (AdcMirror.ADCRESULT2) - 1859;
    AD_output4 = (AdcMirror.ADCRESULT3) - 1860;
    AD_output5 = (AdcMirror.ADCRESULT4) - 1860;
    AD_output6 = (AdcMirror.ADCRESULT5) - 1860;
    AD_output7 = (AdcMirror.ADCRESULT6) - 1865;


    // Conversão dos valores de saída do conversor A/D
    Tensao_A[AMOSTRAS-1]    = KV_PCI*Kad*(float)AD_output1;
    Tensao_B[AMOSTRAS-1]    = KV_PCI*Kad*(float)AD_output2;
    Tensao_C[AMOSTRAS-1]    = KV_PCI*Kad*(float)AD_output3;
    Corrente_A[AMOSTRAS-1]  = KI_PCI*Kad*(float)AD_output4;
    Corrente_B[AMOSTRAS-1]  = KI_PCI*Kad*(float)AD_output5;
    Corrente_C[AMOSTRAS-1]  = KI_PCI*Kad*(float)AD_output6;
    Vcc[AMOSTRAS-1] = KV_PCI*Kad*(float)AD_output7;

    // Testes do sistema de aquisição: Detecção de amplitudes e frequencia dos sinais do lado da linha

    Data_Estimation();

    // Calculo de tensão no capacitor

    V_CAP[1] = V_CAP[0];
    V_CAP[0] = Vcc[AMOSTRAS-1];

    V_CAPF[1] = V_CAPF[0];

    /*
     * Filtragem do sinal de realimentação.
     * O Valor da frequência de corte do filtro passa-baixas é 500 rad/s.
     * */

    V_CAPF[0] = 0.02439*(V_CAP[0] + V_CAP[1]) + 0.9512*V_CAPF[1];

    // Execução do DSOGI-FLL

    DSOGI_FLL(Tensao_A[AMOSTRAS-1], Tensao_B[AMOSTRAS-1], Tensao_C[AMOSTRAS-1]);

    teste_PLL();

    // Transformação vetorial das tensões: ABC para DQO
    Vd_med[1] = Vd_med[0];
    Vq_med[1] = Vq_med[0];
    Vd_med[0] = 0.66666667*( Tensao_A[AMOSTRAS-1]*cos(Teta) + Tensao_B[AMOSTRAS-1]*cos(Teta - 2.094395102) + Tensao_C[AMOSTRAS-1]*cos(Teta + 2.094395102) );
    Vq_med[0] = -0.66666667*( Tensao_A[AMOSTRAS-1]*sin(Teta) + Tensao_B[AMOSTRAS-1]*sin(Teta - 2.094395102) + Tensao_C[AMOSTRAS-1]*sin(Teta + 2.094395102) );

    // Transformação vetorial das correntes: ABC para DQO
    Id_med[1] = Id_med[0];
    Iq_med[1] = Iq_med[0];
    Id_med[0] = 0.66666667*( Corrente_A[AMOSTRAS-1]*cos(Teta) + Corrente_B[AMOSTRAS-1]*cos(Teta - 2.094395102) + Corrente_C[AMOSTRAS-1]*cos(Teta + 2.094395102) );
    Iq_med[0] = -0.66666667*( Corrente_A[AMOSTRAS-1]*sin(Teta) + Corrente_B[AMOSTRAS-1]*sin(Teta - 2.094395102) + Corrente_C[AMOSTRAS-1]*sin(Teta + 2.094395102) );


    // Filtragem e normalização das tensões
    // --------------------------------------
    // Filtragem e normalização das tensões: utiliza-se um filtro digital passa-baixas
    // com frequência de corte de MMM Hz

    Vd_medf[1] = Vd_medf[0];
    Vq_medf[1] = Vq_medf[0];
    Vd_medf[0] = 0.002494*Vd_med[0] + 0.002494*Vd_med[1] + 0.995*Vd_medf[1];
    Vq_medf[0] = 0.002494*Vq_med[0] + 0.002494*Vq_med[1] + 0.995*Vq_medf[1];


    // <---------------- Utilizado pelo professor

    // Filtragem e normalização das correntes
    // --------------------------------------
    // Filtragem e normalização das correntes: utiliza-se um filtro digital passa-baixas
    // com frequência de corte de 159,15 Hz (1000 rad/s)

    //Id_medf[1] = Id_medf[0];
    //Iq_medf[1] = Iq_medf[0];
    //Id_medf[0] = 0.04762*Id_med[0] + 0.04762*Id_med[1] + 0.9048*Id_medf[1];
    //Iq_medf[0] = 0.04762*Iq_med[0] + 0.04762*Iq_med[1] + 0.9048*Iq_medf[1];

    // <---------------- Utilizado pelo professor



    // * Filtragem e normalização das correntes
    // * --------------------------------------
    // * Filtragem e normalização das correntes: utiliza-se um filtro digital passa-baixas
    // * com frequência de corte de 79,57 Hz (500 rad/s)


    Id_medf[1] = Id_medf[0];
    Iq_medf[1] = Iq_medf[0];
    Id_medf[0] = 0.02439*(Id_med[0] + Id_med[1]) + 0.9512*Id_med[1];
    Iq_medf[0] = 0.02439*(Iq_med[0] + Iq_med[1]) + 0.9512*Iq_med[1];

    // Conversão das tensões medidas para pu

    Vd_pu = Vd_medf[0]/(V_base*sqrt(2)/sqrt(3));
    Vq_pu = Vq_medf[0]/(V_base*sqrt(2)/sqrt(3));

    // Conversão das correntes medidas para pu

    Id_pu = Id_medf[0]/I_base;
    Iq_pu = Iq_medf[0]/I_base;

    // Cálculo de potência

    P_med[1] = P_med[0];
    Q_med[1] = Q_med[0];

    P_med[0] = 1.5*(Vd_med[0]*Id_med[0] + Vq_med[0]*Iq_med[0])/S_base;
    Q_med[0] = 1.5*(Vq_med[0]*Id_med[0] - Vd_med[0]*Iq_med[0])/S_base;

    // * Filtragem das potências trifásicas. O filtro passa-baixas têm
    // * frequência de corte em 100 rad/s.


    P_medf[1] = P_medf[0];
    Q_medf[1] = Q_medf[0];

    P_medf[0] = 0.004975*(P_med[0] + P_med[1]) + 0.99*P_medf[1];
    Q_medf[0] = 0.004975*(Q_med[0] + Q_med[1]) + 0.99*Q_medf[1];

    // Controladores de potência

    Erro_P[1] = Erro_P[0];
    Erro_Q[1] = Erro_Q[0];

    Erro_P[0] = -1*(1 - V_CAPF[0]/680);
    Erro_Q[0] = 0 - Q_medf[0];

    Id_ref[1] = Id_ref[0];

    Id_ref[0] = K1*Erro_P[0] - K2*Erro_P[1] + Id_ref[1];
    Id_ref[0] = 0.2; // <--- Valor inserido de corrente

    Iq_ref[1] = Iq_ref[0];

    Iq_ref[0] = K3*Erro_Q[0] - K4*Erro_Q[1] + Iq_ref[1];
    Iq_ref[0] = 0.0; // <-- Valor inserido de corrente

    // Controladores de corrente de eixo direto: calculo do erro e execução do controle PI

    Erro_id[1] = Erro_id[0];
    Erro_id[0] = Id_ref[0] - Id_pu;
    Pi_id[1] = Pi_id[0];
    Pi_id[0] = K5*Erro_id[0] - K6*Erro_id[1] + Pi_id[1];

    Ed_inv = Pi_id[0] +  Vd_pu - (L_pu*Iq_pu); // Tensão de saída do inversor

    // Controladores de corrente de eixo em quadratura: calculo do erro e execução do controle PI
    Erro_iq[1] = Erro_iq[0];
    Erro_iq[0] = -Iq_ref[0] - Iq_pu;
    Pi_iq[1] = Pi_iq[0];
    Pi_iq[0] = K5*Erro_iq[0] - K6*Erro_iq[1] + Pi_iq[1];

    Eq_inv = Pi_iq[0] +  Vq_pu + (L_pu*Id_pu); // Tensão de saída do inversor



    // Transformação vetorial inversa
    //InvFaseA = 0.66666667*( Ed_inv*cos(Teta) - Eq_inv*sin(Teta) );
    //InvFaseB = 0.66666667*( Ed_inv*cos(Teta - 2.094395102) - Eq_inv*sin(Teta - 2.094395102) );
    //InvFaseC = 0.66666667*( Ed_inv*cos(Teta + 2.094395102) - Eq_inv*sin(Teta + 2.094395102) );

    InvFaseA = 0.95*cos(Teta);
    InvFaseB = 0.95*cos(Teta - 2.094395102);
    InvFaseC = 0.95*cos(Teta + 2.094395102);

    // Determinação dos pulsos de chaveamento
    Change_ePWM(InvFaseA,InvFaseB,InvFaseC);

    // Reset ADC Sequencer1 (Register ADCCTRL2)
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;

    // Clear Interrupt Flag ADC Sequencer 1 (Register ADCST)
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

    // Acknowledge PIE Interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

/*void SpaceVectorPWM(void)
{


}
*/

void DSOGI_FLL(float Va, float Vb, float Vc)
{
/*
 * Esta função implementa o método de sincronização conhecido por DSOGI-FLL
 */

    float V_alfa = 0, V_beta = 0;
    float Ev_alfa = 0, kEv_alfa = 0, qV_alfa = 0;
    float Ev_beta = 0, kEv_beta = 0, qV_beta = 0;

    // Transformação vetorial (abc -> ab0)
    V_alfa = 0.66666667*(Va - 0.5*Vb - 0.5*Vc);
    V_beta = 0.66666667*(0.86602540*Vb - 0.86602540*Vc);

    // Adaptative filter (SOGI) - Componente alfa
    // ------------------------------------------
    V_alfa_inter[1]= V_alfa_inter[0];
    V_alfa_int[1] = V_alfa_int[0];
    V_alfa_out[1] = V_alfa_out[0];

    Ev_alfa = V_alfa - V_alfa_out[0];
    kEv_alfa = k*Ev_alfa;
    qV_alfa = V_alfa_int[0]*Freq_in;
    V_alfa_inter[0] = Freq_in*(kEv_alfa - qV_alfa);

    // Integração: V_alfa_inter -> V_alfa_out
    V_alfa_out[0] = (PERI_AMOSTRAGEM/2)*V_alfa_inter[0] + (PERI_AMOSTRAGEM/2)*V_alfa_inter[1] + V_alfa_out[1];

    // Integração: V_alfa_out -> V_alfa_int
    V_alfa_int[0] = (PERI_AMOSTRAGEM/2)*V_alfa_out[0] + (PERI_AMOSTRAGEM/2)*V_alfa_out[1] + V_alfa_int[1];


    // Adaptative filter (SOGI) - Componente beta
    // ------------------------------------------
    V_beta_inter[1]= V_beta_inter[0];
    V_beta_int[1] = V_beta_int[0];
    V_beta_out[1] = V_beta_out[0];

    Ev_beta = V_beta - V_beta_out[0];
    kEv_beta = k*Ev_beta;
    qV_beta = V_beta_int[0]*Freq_in;
    V_beta_inter[0] = Freq_in*(kEv_beta - qV_beta);

    // Integração: V_alfa_inter -> V_alfa_out
    V_beta_out[0] = (PERI_AMOSTRAGEM/2)*V_beta_inter[0] + (PERI_AMOSTRAGEM/2)*V_beta_inter[1] + V_beta_out[1];

    // Integração: V_alfa_out -> V_alfa_int
    V_beta_int[0] = (PERI_AMOSTRAGEM/2)*V_beta_out[0] + (PERI_AMOSTRAGEM/2)*V_beta_out[1] + V_beta_int[1];


    // Positive and negative sequence voltage calculation
    V_alfa_P = 0.5*V_alfa_out[0] - 0.5*qV_beta;
    V_alfa_N = 0.5*V_alfa_out[0] + 0.5*qV_beta;
    V_beta_P = 0.5*V_beta_out[0] + 0.5*qV_alfa;
    V_beta_N = 0.5*V_beta_out[0] - 0.5*qV_alfa;

    // FLL Normalization
    FLL_int[1] = FLL_int[0];
    if(first_time == 0)
    {
        FLL_int[0] = 0.0000000001;
        first_time++;
    }
    else{
        FLL_int[0] = ( k*Freq_in/( V_alfa_P*V_alfa_P + V_beta_P*V_beta_P ) )*( -Tau*(Ev_alfa*qV_alfa + Ev_beta*qV_beta) );
    }


    // Integração: FLL_int -> FLL_Freq
    FLL_Freq[1] = FLL_Freq[0];
    FLL_Freq[0] = (PERI_AMOSTRAGEM/2)*FLL_int[0] + (PERI_AMOSTRAGEM/2)*FLL_int[1] + FLL_Freq[1];

    Freq_in = FLL_Freq[0] + Freq_Rede;

    // Phase angle calculation: atan retorna valores situados entre -pi/2 e pi/2
    Teta = atan(V_beta_P/V_alfa_P);

    // Ajuste do ângulo TETA, para situar a variável entre 0 e 2*PI
    if (V_alfa_P < 0)
        Teta = Teta + PI;
    else if (V_alfa_P > 0 && V_beta_P <0)
        Teta = Teta + 2*PI;
}

//void Transformation_ABC_2_DQ0(float FaseA, float FaseB, float FaseC, float Teta)
//{
    /*
     * Esta função implementa a transformação de Park, realizando a transformação do sistema
     *  ABC para o sistema DQO. As equações que modelam esta transformação são:
     *      Vd = (2/3)*(Va*cos(Teta) + Vb*cos(Teta - 2*pi/3) + Vc*cos(Teta + 2*pi/3)
     *      Vq = (2/3)*(Va*sin(Teta) + Vb*sin(Teta - 2*pi/3) + Vc*sin(Teta + 2*pi/3)
     *      V0 = (2/3)*(0.5*Va + 0.5*Vb + 0.5*Vc)
     *
     * Variáveis de entrada da função:
     *          float FaseA: sinal instantâneo da Fase A;
     *          float FaseB: sinal instantâneo da Fase B;
     *          float FaseC: sinal instantâneo da Fase C;
     *          float Teta: ângulo de referência para a transformação.
     *
     *  Variáveis de saída da função:
     *
     */
//  float Daxis = 0, Qaxis = 0, Zeroaxis = 0;

//  Daxis = 0.66666667*( FaseA*cos(Teta) + FaseB*cos(Teta - 2.094395102) + FaseC*cos(Teta + 2.094395102) );
//  Qaxis = -0.66666667*( FaseA*sin(Teta) + FaseB*sin(Teta - 2.094395102) + FaseC*sin(Teta + 2.094395102) );
//  Zeroaxis = 0.66666667*( 0.5*FaseA + 0.5*FaseB + 0.5*FaseC );

//}

//void Transformation_DQ0_2_ABC(float Daxis, float Qaxis, float Zeroaxis, float Teta)
//{
    /*
     * Esta função implementa a transformação inversa de Park, realizando a transformação do sistema
     *  DQ0 para o sistema ABC. As equações que modelam esta transformação são:
     *      Va = (2/3)*(Vd*cos(Teta) + Vq*sin(Teta) + V0)
     *      Vb = (2/3)*(Vd*cos(Teta - 2*pi/3) + Vq*sin(Teta - 2*pi/3) + V0)
     *      Vc = (2/3)*(Vd*cos(Teta + 2*pi/3) + Vq*sin(Teta + 2*pi/3) + V0)
     *
     * Variáveis de entrada da função:
     *          float Daxis: componente de eixo direto
     *          float Qaxis: componente de eixo em quadratura
     *          float Zeroaxis: componente de sequência zero
     *          float Teta: ângulo de referência para a transformação.
     *
     *  Variáveis de saída da função:
     *
     */
//  float FaseA = 0, FaseB = 0, FaseC = 0;

//  FaseA = 0.66666667*(Daxis*cos(Teta) + Qaxis*sin(Teta) + Zeroaxis);
//  FaseB = 0.66666667*(Daxis*cos(Teta - 2.094395102) + Qaxis*sin(Teta - 2.094395102) + Zeroaxis);
//  FaseC = 0.66666667*(Daxis*cos(Teta + 2.094395102) + Qaxis*sin(Teta + 2.094395102) + Zeroaxis);

//}

void teste_PLL(void){
    /*
     * Esta função verifica o status da PLL com relação a fase A da rede.
     * A verificação é realizada através da GPIO12 do DSP.
     * Quando o ângulo é menor que PI, ou a tensão Vpll é maior que zero, a GPIO12 é 1.
     * No contrário, a GPIO12 é 0.
     * */
    Vpll = cos(Teta);

    if (Vpll >= 0.0){
        // Colocar a GPIO em 3.3 V
        GpioDataRegs.GPASET.bit.GPIO10 = 1;
    }
    else{
        // Colocar a GPIO em 0 V
        GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    }
}

void Data_Estimation(void){
    if (V_A == 0){
        V_A = Tensao_A[AMOSTRAS-1];
    }
    else if (Tensao_A[AMOSTRAS-1] > V_A){
        V_A = Tensao_A[AMOSTRAS-1];
    }

    //Id_medf[1] = Id_medf[0];
    //Iq_medf[1] = Iq_medf[0];
    //Id_medf[0] = 0.04762*Id_med[0] + 0.04762*Id_med[1] + 0.9048*Id_medf[1];
    //Iq_medf[0] = 0.04762*Iq_med[0] + 0.04762*Iq_med[1] + 0.9048*Iq_medf[1];

    I_Af[1] = I_Af[0];
    I_Af[0] = 0.02439*(Corrente_A[AMOSTRAS-1] + Corrente_A[AMOSTRAS-2]) + 0.9512*I_Af[1];

    if (I_A == 0){
        I_A = I_Af[0];
    }
    else if (I_Af[0] > I_A){
        I_A = I_Af[0];
    }

    I_Bf[1] = I_Bf[0];
    I_Bf[0] = 0.02439*(Corrente_B[AMOSTRAS-1] + Corrente_B[AMOSTRAS-2]) + 0.9512*I_Bf[1];

        if (I_B == 0){
            I_B = I_Bf[0];
        }
        else if (I_Bf[0] > I_B){
            I_B = I_Bf[0];
        }

    I_Cf[1] = I_Cf[0];
    I_Cf[0] = 0.02439*(Corrente_C[AMOSTRAS-1] + Corrente_C[AMOSTRAS-2]) + 0.9512*I_Cf[1];

        if (I_C == 0){
            I_C = I_Cf[0];
        }
        else if (I_Cf[0] > I_C){
            I_C = I_Cf[0];
        }

}

void Setup_ePWM(void){
        // Configures three ePWMs for use as a SPWM

        // Configuration of EPwm1
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // Set multiplexer register line GPIO0 to enable ePWM1A as output signal.
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // Set multiplexer register line GPIO1 to enable ePWM1B
        EDIS;

        EPwm1Regs.TBCTL.bit.CTRMODE = 2; // TBCTL - Time Base Control Register
                                         // Field CTRMODE - Counter Mode
                                         // 00 = count up, 01 = count down, 10 = count
                                         // up and down and 11 = stop - freeze (default)
        EPwm1Regs.TBCTL.bit.CLKDIV = 0; // Divides by 1
        EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Divides by 1

        // The equation to define the PWM period is
        // TBPRD = 0.5*((f_sysclkout)/(f_pwm * CLKDIV * HSPCLKDIV))
        // The system has f_sysclkout = 150 MHz. We want the f_pwm frequency of 2.5 kHz.
        // Using CLKDIV = 1 and HSPCLKDIV = 1, we can obtain
        // TBPRD = 0.5*(150*10^6)/(2.5*10^3) = 30000

        EPwm1Regs.TBPRD = 21429-1;//30000-1;

        // We would like to set ePWM1A to 1 on "CMPA - up match" and to
        // clear ePWM1A on event "CMPA - down math".
        // Register AQCTLA
        // CAD (bits 7-6): Action when CTR = CMPA on down count
        // CAU (bits 5-4): Action when CTR = CMPA on up count
        // 00 = do nothing (action disabled), 01 = clear (low), 10 = set (high), 11 = toogle
        // CAD = 1; CAU = 2;
        EPwm1Regs.AQCTLA.bit.CAD = 1;
        EPwm1Regs.AQCTLA.bit.CAU = 2;

        // Set register CMPA and CMPB to initially generate a
        // pulse width of 50%
        EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2;

        // Setup a delay time between outA and outB of 10 microseconds

        EPwm1Regs.DBRED = 750;
        EPwm1Regs.DBFED = 750;
        EPwm1Regs.DBCTL.bit.IN_MODE = 0;
        EPwm1Regs.DBCTL.bit.POLSEL = 2;
        EPwm1Regs.DBCTL.bit.OUT_MODE = 3;

        // Configuration of EPwm2
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // Set multiplexer register line GPIO2 to enable ePWM2A as output signal.
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // Set multiplexer register line GPIO3 to enable ePWM2B as output signal
        EDIS;
        EPwm2Regs.TBCTL.bit.CTRMODE = 2;
        EPwm2Regs.TBCTL.bit.CLKDIV = 0;
        EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
        EPwm2Regs.TBPRD = 21429-1;//30000-1;
        EPwm2Regs.AQCTLA.bit.CAD = 1;
        EPwm2Regs.AQCTLA.bit.CAU = 2;
        EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD/2;
        EPwm2Regs.DBRED = 750;
        EPwm2Regs.DBFED = 750;
        EPwm2Regs.DBCTL.bit.IN_MODE = 0;
        EPwm2Regs.DBCTL.bit.POLSEL = 2;
        EPwm2Regs.DBCTL.bit.OUT_MODE = 3;

        // Configuration of EPwm2
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // Set multiplexer register line GPIO4 to enable ePWM3A as output signal.
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; // Set multiplexer register line GPIO5 to enable ePWM3B as output signal
        EDIS;
        EPwm3Regs.TBCTL.bit.CTRMODE = 2;
        EPwm3Regs.TBCTL.bit.CLKDIV = 0;
        EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
        EPwm3Regs.TBPRD = 21429-1;//30000-1;
        EPwm3Regs.AQCTLA.bit.CAD = 1;
        EPwm3Regs.AQCTLA.bit.CAU = 2;
        EPwm3Regs.CMPA.half.CMPA = EPwm3Regs.TBPRD/2;
        EPwm3Regs.DBRED = 750;
        EPwm3Regs.DBFED = 750;
        EPwm3Regs.DBCTL.bit.IN_MODE = 0;
        EPwm3Regs.DBCTL.bit.POLSEL = 2;
        EPwm3Regs.DBCTL.bit.OUT_MODE = 3;

}

void Change_ePWM(float InvFaseA, float InvFaseB, float InvFaseC){
    // Esta função converte a onda modulante que está entre -1 e 1
    // em um valor inteiro e insere em cada registrador CMPA para que
    // estes possam ser comparados com os registradores TBPRD de cada ePWM
    unsigned int value1, value2, value3;
    value1 = (InvFaseA + 1.0)*(EPwm1Regs.TBPRD/2);
    value2 = (InvFaseB + 1.0)*(EPwm1Regs.TBPRD/2);
    value3 = (InvFaseC + 1.0)*(EPwm1Regs.TBPRD/2);

    value1 = (unsigned int)value1;
    value2 = (unsigned int)value2;
    value3 = (unsigned int)value3;

    //EPwm1Regs.CMPA.half.CMPA = (EPwm1Regs.TBPRD/2);
    //EPwm2Regs.CMPA.half.CMPA = (EPwm2Regs.TBPRD/2);
    //EPwm3Regs.CMPA.half.CMPA = (EPwm3Regs.TBPRD/2);
    EPwm1Regs.CMPA.half.CMPA = value1;
    EPwm2Regs.CMPA.half.CMPA = value2;
    EPwm3Regs.CMPA.half.CMPA = value3;

}


//===========================================================================
// End of SourceCode.
//===========================================================================
