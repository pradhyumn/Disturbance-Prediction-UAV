#include <iostream>
using namespace std;

int main() {
	int i, j, k, arraysize_val;
	float P_mat[3][3], D_mat[3][3], M_mat[3][3];          // P, D, M matrices
	float m_val = 0.5, t_end_val = 30.0, h_val = 0.01;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			if (i == j)
			{
				P_mat[i][j] = 2000.0;
				M_mat[i][j] = m_val;
			}
			else
			{
				P_mat[i][j] = 0;
				M_mat[i][j] = 0;
			}

			if (i == 0 && j == 0)
			{
				D_mat[i][j] = 0.2 * 12;
			}
			else if (i == 1 && j == 1)
			{
				D_mat[i][j] = 0.2 * 13;
			}
			else if (i == 2 && j == 2)
			{
				D_mat[i][j] = 0.2 * 14;
			}
			else
			{
				D_mat[i][j] = 0.0;
			}
		}
	}
	arraysize_val = (t_end_val / h_val) + 1;
	/***************--------------------**************************************/

	/*************************************************************************/
	float Rhat_mat[3][3][arraysize_val];             // Rhat matrix
	for (i = 0; k < arraysize_val; k++)
	{
		for (i = 0; i < 3; i++)
		{
			for (i = 0; j < 3; j++)
			{
				Rhat_mat[i][j][k] = 0.0;
			}
		}
	}

	float      om_mat[3][arraysize_val];             // Omega matrix
	float betaerr_VAE[3][arraysize_val];             // beta error matrix
	for (i = 0; i < 3; i++)
	{
		for (i = 0; j < arraysize_val; i++)
		{
			om_mat[i][j] = 0.0;
			betaerr_VAE[i][j] = 0.0;
		}
	}
	float att_err_mat[1][arraysize_val];
	float     Phi_VAE[1][arraysize_val];
	for (i = 0; i < 1; i++)
	{
		for (i = 0; j < arraysize_val; i++)
		{
			att_err_mat[i][j] = 0.0;
			Phi_VAE[i][j] = 0.0;
		}
	}
	/*----------------EP2C-------------*/

	/*---------------------------------*/
	bhat_VAE

}