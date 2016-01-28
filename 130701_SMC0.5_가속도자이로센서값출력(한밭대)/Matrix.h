
void mtx_mul_2x2(double ANS[2][2],double A[2][2],double B[2][2]);
void mtx_mul_2x1(double ANS[2],double A[2][2],double B[2]);
void mtx_add_2x2(double ANS[2][2],double A[2][2],double B[2][2],double s);
void mtx_add_2x1(double ANS[2],double A[2],double B[2],double s);
void mtx_inv_2x2(double ANS[2][2],double A[2][2]);

void mtx_mul_3x3(double ANS[3][3],double A[3][3],double B[3][3]);
void mtx_mul_3x1(double ANS[3],double A[3][3],double B[3]);
void mtx_add_3x3(double ANS[3][3],double A[3][3],double B[3][3],double s);
void mtx_add_3x1(double ANS[3],double A[3],double B[3],double s);
void mtx_inv_3x3(double ANS[3][3],double a[3][3]);

void mtx_mul_10x10(double ANS[10][10],double A[10][10],double B[10][10]);
void mtx_mul_10x1(double ANS[10],double A[10][10],double B[10]);
void mtx_add_10x10(double ANS[10][10],double A[10][10],double B[10][10],double s);
void mtx_add_10x1(double ANS[10],double A[10],double B[10],double s);

void mtx_mul_4x4(double ANS[4][4],double A[4][4],double B[4][4]);
int mtx_inv_4x4(double ANS[4][4], double A[4][4]);
double mtx_det_4x4( double A[4][4]);
void mtx_inv_6x6(double inverse[6][6],double matrix[6][6]);
