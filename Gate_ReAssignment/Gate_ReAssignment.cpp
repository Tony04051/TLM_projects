#include<iostream>
#include <set>
#include <string>
#include"gurobi_c++.h"
using namespace std;

int main()
{
	/*set*/
	set <int> ID;	//確定性航班集合
	set <int> IS;	//不確定性航班集合
	

	/*params*/
	const int I = 4;	//所有航班數量
	const int K0 = 2;	//所有登機門，0為停機坪
	int A[I];	//航班i的原訂抵達時間。
	int W;	//更換登機門的懲罰值。
	int B;	//指派至停機坪的懲罰值。
	int S[I];	//航班i的服務時間。
	int U[I];	//航班i的更新後抵達時間
	int Y[I][K0];	//航班i指派至登機門k的事前指派。
	//int M		不知道這是什麼

	/*gurobi*/
	try
	{
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		string name = "";

		/*variables*/
		GRBVar y[I][K0]; //若 = 1代表航班i使用登機門k
		GRBVar t[I];	//航班i在登機門的開始作業時間。
		GRBVar z[I];	//若=1代表航班i使用之登機門與事前指派不同

		//y[i][k], 若=1代表航班i使用登機門k
		for (int i = 0; i < I; i++)
		{
			for (int j = 0; j < K0; j++)
			{
				name = "y_" + to_string(i) + "_" + to_string(j);
				y[i][j] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
			}
		}
		cout << "y sucess" << endl;

		//t[i], 航班i在登機門的開始作業時間。
		for (int i = 0; i < I; i++)
		{
			name = "t_" + to_string(i);
			t[i] = model.addVar(0.0, GRB_INFINITY, 0, GRB_CONTINUOUS, name);
		}
		cout << "t success" << endl;
			
		//z[i], 若=1代表航班i使用之登機門與事前指派不同
		for (int i = 0; i < I; i++)
		{
			name = "z_" + to_string(i);
			z[i] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
		}
		cout << "z success" << endl;

		GRBLinExpr sum = 0;


		/*ojective*/
		for (int i = 0; i < I; i++)
			sum += t[i] - A[i] + W*z[i] + B*y[i][0];
			
		model.setObjective(sum, GRB_MINIMIZE);

		/*constraints*/
		//(19)
		for (int i = 0; i < I; i++)
		{
			sum = 0;
			for (int k = 0; k < K0; k++)
			{
				sum += y[i][k];
			}
			name = "(19)_" + to_string(i);
			model.addConstr(sum == 1, name);
		}
		cout << "(19) sucess" << endl;
		
		//(20)
		//constraint乘上的M不知道是什麼
		for (int i = 0; i < I; i++)
		{
			for (int j = 0; j < I; j++)
			{
				if (U[i] < U[j])
				{
					for (int k = 0; k < K0; k++)
					{
						name = "(20)_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k);
						model.addConstr(t[i] + S[i] - t[j] <= (2 - y[i][k] - y[j][k]), name);
					}
				}
			}
		}
		cout << "(20) success" << endl;
		
		//(21)
		for (int i = 0; i < I; i++)
		{
			name = "(21)" + to_string(i);
			model.addConstr(U[i] <= t[i], name);
		}
		cout << "(21) success" << endl;
		
		//(22)
		for (int i = 0; i < I; i++)
		{
			sum = 0;
			for (int k = 0; k < K0; k++)
			{
				sum += y[i][k] * (1 - Y[i][k]);
			}
			name = "(22) success" + to_string(i);
			model.addConstr(z[i] >= sum, name);
		}
		cout << "(22) success" << endl;



		model.optimize();
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

	system("pause");
	return 0;
}