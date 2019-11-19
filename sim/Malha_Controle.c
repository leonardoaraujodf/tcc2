#include <math.h>

#define V_REF_CC 450.0
static float V_CAP[2] = {0,0}, V_CAPF[2] = {0,0};
float Va = 0, Vb = 0, Vc = 0, Ia = 0, Ib = 0, Ic = 0, Vcc = 0, Teta = 0;
static float Vd_med[2] = {0, 0}, Vq_med[2] = {0, 0}, Vd_medf[2] = {0, 0}, Vq_medf[2] = {0, 0};
static float Id_med[2] = {0, 0}, Iq_med[2] = {0, 0}, Id_medf[2] = {0, 0}, Iq_medf[2] = {0, 0};
float V_base = 380.0;
float I_base = 15.193;
float S_base =  10000.0;
// Indutância do filtro de saída, em pu, para cálculo dos termos de acoplamento entre malhas de controle
float L_pu = 0.07832225;//(376.99111841*L_Filt)/(Z_base) //0.078624099722992; //+ 0.0003; // O valor 0.0003 refere-se a impedância do transformador em pu
static float Vd_pu = 0, Vq_pu = 0;
static float Id_pu = 0, Iq_pu = 0;
static float P_med[2] = {0,0}, Q_med[2] = {0,0};
static float P_medf[2] = {0,0}, Q_medf[2] = {0,0};
static float Erro_P[2] = {0,0}, Erro_Q[2] = {0,0};
static float Id_ref[2] = {0, 0}, Iq_ref[2] = {0, 0};
static float Erro_id[2] = {0, 0}, Erro_iq[2] = {0, 0};
static float Pi_id[2] = {0, 0}, Pi_iq[2] = {0, 0};
static float Ed_inv = 0, Eq_inv = 0, InvFaseA = 0, InvFaseB = 0, InvFaseC = 0;
static int contador_malha_controle = 0;
float limite_operacional = 0;
float corrente_requerida = 0;

// Ganhos dos controladores PI de Potência Ativa
/*
 * Ganhos para o capacitor de 6 mF
 * K1 = 0.7523, K2 = 0.7477
 * K3 = 0.739, K4 = 0.7269
 * K5 = 0.7358, K6 = 0.7242
 *
 * Ganhos para o capacitor de 940 uF
 * K1 = 0.1184, K2 = 0.1176
 * K3 = 0.5081, K4 = 0.4939
 * K5 = 0.8104, K6 = 0.7957
 *
 * Ganhos utilizados na tese do Leonardo
 * K1 = 0.8895, K2 = 0.8812;
 * K3 = 0.7693, K4 = 0.7611;
 * K5 = 0.8104, K6 = 0.7957;
 * */

float K1 = 0.8895, K2 = 0.8812;

// Ganhos dos controladores PI de Potência Reativa
float K3 = 0.7693, K4 = 0.7611;

// Ganhos dos controladores PI de Corrente
float K5 = 0.8104, K6 = 0.7611;

Va = x1;
Vb = x2;
Vc = x3;
Ia = x4;
Ib = x5;
Ic = x6;
Vcc = x7;
Teta = x8;

if (contador_malha_controle > 3000-1){

	
	// Calculo de tensão no capacitor
	V_CAP[1] = V_CAP[0];
	V_CAP[0] = Vcc;
	
	V_CAPF[1] = V_CAPF[0];
	V_CAPF[0] = 0.00624*(V_CAP[0] + V_CAP[1]) + 0.98751*V_CAPF[1];
	
	// Transformação vetorial das tensões: ABC para DQO
	Vd_med[1] = Vd_med[0];
	Vq_med[1] = Vq_med[0];
	Vd_med[0] = 0.66666667*( Va*cos(Teta) + Vb*cos(Teta - 2.094395102) + Vc*cos(Teta + 2.094395102) );
	Vq_med[0] = -0.66666667*( Va*sin(Teta) + Vb*sin(Teta - 2.094395102) + Vc*sin(Teta + 2.094395102) );
	
	// Transformação vetorial das correntes: ABC para DQO
	Id_med[1] = Id_med[0];
	Iq_med[1] = Iq_med[0];
	Id_med[0] = 0.66666667*( Ia*cos(Teta) + Ib*cos(Teta - 2.094395102) + Ic*cos(Teta + 2.094395102) );
	Iq_med[0] = -0.66666667*( Ia*sin(Teta) + Ib*sin(Teta - 2.094395102) + Ic*sin(Teta + 2.094395102) );
	
	// Filtragem e normalização das tensões
	Vd_medf[1] = Vd_medf[0];
	Vq_medf[1] = Vq_medf[0];
	Vd_medf[0] = 0.002494*Vd_med[0] + 0.002494*Vd_med[1] + 0.995*Vd_medf[1];
	Vq_medf[0] = 0.002494*Vq_med[0] + 0.002494*Vq_med[1] + 0.995*Vq_medf[1];
	
	// Filtragem e normalização das correntes
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
	
	Erro_P[0] = -1*(1 - V_CAPF[0]/V_REF_CC);
	Erro_Q[0] = 0 - Q_medf[0];
	
	Id_ref[1] = Id_ref[0];
	
	Id_ref[0] = K1*Erro_P[0] - K2*Erro_P[1] + Id_ref[1];
	
	Iq_ref[1] = Iq_ref[0];
	Iq_ref[0] = K3*Erro_Q[0] - K4*Erro_Q[1] + Iq_ref[1];

	// Análise dos limites operacionais do conversor
	
	limite_operacional = (sqrt(2)*S_base)/(sqrt(3)*V_base);
	limite_operacional = limite_operacional*limite_operacional;
	corrente_requerida = Id_ref[0]*Id_ref[0] + Iq_ref[0]*Iq_ref[0];
	
	if(corrente_requerida > limite_operacional){
		Iq_ref[0] = sqrt(limite_operacional - Id_ref[0]*Id_ref[0]);
	}

	// Controladores de corrente de eixo direto: calculo do erro e execução do controle PI
	
	Erro_id[1] = Erro_id[0];
	Erro_id[0] = Id_ref[0] - Id_pu;
	Pi_id[1] = Pi_id[0];
	Pi_id[0] = K5*Erro_id[0] - K6*Erro_id[1] + Pi_id[1];
	
	// Tensão de saída do inversor
	Ed_inv = Pi_id[0] +  Vd_pu - (L_pu*Iq_pu); 
	
	// Controladores de corrente de eixo em quadratura: calculo do erro e execução do controle PI
	
	Erro_iq[1] = Erro_iq[0];
	Erro_iq[0] = -Iq_ref[0] - Iq_pu;
	Pi_iq[1] = Pi_iq[0];
	Pi_iq[0] = K5*Erro_iq[0] - K6*Erro_iq[1] + Pi_iq[1];
	
	// Tensão de saída do inversor
	Eq_inv = Pi_iq[0] +  Vq_pu + (L_pu*Id_pu);
	
	// Transformação vetorial inversa
	InvFaseA = 0.66666667*( Ed_inv*cos(Teta) - Eq_inv*sin(Teta) );
	InvFaseB = 0.66666667*( Ed_inv*cos(Teta - 2.094395102) - Eq_inv*sin(Teta - 2.094395102) );
	InvFaseC = 0.66666667*( Ed_inv*cos(Teta + 2.094395102) - Eq_inv*sin(Teta + 2.094395102) );
	
	y1 = InvFaseA;
	y2 = InvFaseB;
	y3 = InvFaseC;
	y4 = contador_malha_controle;
}
else{
	contador_malha_controle = contador_malha_controle + 1;
	InvFaseA = 0;
	InvFaseB = 0;
	InvFaseC = 0;
	y1 = InvFaseA;
	y2 = InvFaseB;
	y3 = InvFaseC;
	y4 = contador_malha_controle;
}


