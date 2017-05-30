#include "cdMatrix.h"

//cdMatrix equalize operator
cdMatrix::cdMatrix()
{
	Data = NULL;
	rows = 0; columns = 0;
}

cdMatrix::cdMatrix(int _rows, int _columns)
{
#ifdef DEBUGGER
	cout << "Initiator(int _rows, int _columns)" << endl;
#endif

	rows = _rows;
	columns = _columns;

	Data = (float**) new float*[_rows];
	for (int l = 0; l < _rows; l++)
		Data[l] = (float*) new float[_columns];
}

cdMatrix::cdMatrix(const cdMatrix &_a)
{
#ifdef DEBUGGER
	cout << "Initiator Matrix(const Matrix &_a)" << endl;
#endif

	columns = _a.columns;
	rows = _a.rows;

	Data = new float*[rows];
	for (int i = 0; i < rows; i++)
		Data[i] = new float[columns];

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			this->Data[i][j] = _a.Data[i][j];
		}
	}
}

cdMatrix::~cdMatrix()
{
#ifdef DEBUGGER
	cout << "Destructor" << endl;
#endif

	if (Data != NULL)
	{
		for (int i = 0; i < columns; i++)
			delete[] Data[i];

		delete[]Data;
	}
}

cdMatrix cdMatrix::operator =(const cdMatrix &_a)
{
#ifdef DEBUGGER
	cout << "=" << endl;
#endif

	if (rows == 0 || columns == 0)
	{
		columns = _a.columns;
		rows = _a.rows;

		Data = new float*[rows];
		for (int i = 0; i < rows; i++)
			Data[i] = new float[columns];
	}

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = _a.Data[i][j];
		}
	}

	return *this;
}


//cdMatrix multiply operator
cdMatrix cdMatrix::operator *(const cdMatrix &_a) const
{
	cdMatrix result(rows, _a.columns);

	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	for (int i = 0; i < result.rows; i++)
	{
		for (int j = 0; j < result.columns; j++)
		{
			result.Data[i][j] = 0;
			for (int k = 0; k < result.rows; k++)
			{
				(result.Data[i][j]) += ((Data[i][k])*(_a.Data[k][j]));
			}
		}
	}

#ifdef DEBUGGER
	result.Show_matrix();
#endif
	return result;
}

// cdMatrix multiply operator
cdMatrix cdMatrix::operator *(float mult)const
{
	cdMatrix result(rows, columns);

	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	for (int i = 0; i < result.rows; i++)
		for (int j = 0; j < result.columns; j++)
			result.Data[i][j] = (Data[i][j])*mult;

	return result;
}

// cdMatrix divide by float variable
cdMatrix cdMatrix::operator /(float divide)const
{
	cdMatrix result(rows, columns);

	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	for (int i = 0; i < result.rows; i++)
		for (int j = 0; j < result.columns; j++)
			result.Data[i][j] = (Data[i][j]) / divide;

	return result;
}

//cdMatrix Plus operator
cdMatrix cdMatrix::operator +(const cdMatrix &_a)const
{
	if (rows != _a.rows || columns != _a.columns)
	{
		cout << "Check your input matrix, this operation cannot be executed" << endl;
		exit(0);
	}

	cdMatrix result(rows, columns);

	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	for (int i = 0; i < result.rows; i++)
		for (int j = 0; j < result.columns; j++)
			result.Data[i][j] = Data[i][j] + _a.Data[i][j];

	return result;
}

//cdMatrix minus operator
cdMatrix cdMatrix::operator -()
{
	cdMatrix result(rows, columns);

	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	for (int i = 0; i < result.rows; i++)
		for (int j = 0; j < result.columns; j++)
			result.Data[i][j] = -Data[i][j];

	return result;
}

//cdMatrix minus operator
cdMatrix cdMatrix::operator -(const cdMatrix &_a)const
{
	if (rows != _a.rows || columns != _a.columns)
	{
		cout << "Check your input matrix, this operation cannot be executed" << endl;
		exit(0);
	}

	cdMatrix result(rows, columns);

	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	for (int i = 0; i < result.rows; i++)
		for (int j = 0; j < result.columns; j++)
			result.Data[i][j] = Data[i][j] - _a.Data[i][j];

	return result;
}

//Dot product
float cdMatrix::operator ^(const cdMatrix &_a) const
{
	if (columns != 1 || _a.columns != 1)
	{
		cout << "Check your input matrix, this operation cannot be executed" << endl;
		exit(0);
	}

	float Dot_product = 0;
	for (int i = 0; i < rows; i++)
		Dot_product += ((Data[i][0])*(_a.Data[i][0]));

	return Dot_product;
}

//Input matrix data, only 4 columns matrix
void cdMatrix::Input_data(float _Data[4][4])
{
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < columns; j++)
			Data[i][j] = _Data[i][j];
}

//Input matrix data, only 3 columns matrix
void cdMatrix::Input_data(float _Data[3][3])
{
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < columns; j++)
			Data[i][j] = _Data[i][j];
}

//Input matrix data, only 1 columns matrix
void cdMatrix::Input_data(float _Data[][1])
{
	for (int i = 0; i < rows; i++)
		Data[i][0] = _Data[i][0];
}

void cdMatrix::Input_data(float *_Data)
{
	for (int i = 0; i < rows; i++)
		Data[i][0] = _Data[i];
}

void cdMatrix::Input_data(float x, float y, float z)
{
	if (rows != 3 || columns != 1)
		return;

	Data[0][0] = x;
	Data[1][0] = y;
	Data[2][0] = z;
}

void cdMatrix::Input_data(float x, float y, float z, float t)
{
	if (columns != 1)
		return;

	Data[0][0] = x;
	Data[1][0] = y;
	Data[2][0] = z;
	Data[3][0] = t;
}

void cdMatrix::Output_data(float **Output_data)
{
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < columns; j++)
			Output_data[i][j] = Data[i][j];
}

void cdMatrix::Output_data(float Output_data[][1])
{
	for (int i = 0; i < rows; i++)
		Output_data[i][0] = Data[i][0];
}

void cdMatrix::Output_data(float *Output_data)
{
	for (int i = 0; i < rows; i++)
		Output_data[i] = Data[i][0];
}

float cdMatrix::GetElement(int i, int j)
{
	return Data[i][j];
}

void cdMatrix::SetMatDim(int _rows, int _cols)
{
	rows = _rows;
	columns = _cols;

	Data = (float**) new float*[_rows];
	for (int l = 0; l < _rows; l++)
		Data[l] = (float*) new float[_cols];
}

void cdMatrix::Transpose_matrix()
{
	if (columns == 1)
	{
		cout << "This matrix cannot be inversed in this way" << endl;
		return;
	}

	float **temp = (float**) new float*[rows];
	for (int l = 0; l < columns; l++)
		temp[l] = (float*) new float[columns];

	for (int i = 0; i < rows; i++)
		for (int j = 0; j < columns; j++)
			temp[i][j] = Data[j][i];

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = temp[i][j];
		}
	}

	for (int i = 0; i < rows; i++)
		delete[]temp[i];
	delete[]temp;
}

void cdMatrix::Show_matrix()
{
	cout << "================================" << endl;
	for (int i = 0; i < rows; i++)
	{
		cout << "|  ";
		for (int j = 0; j < columns; j++)
		{
			cout.precision(9);
			cout << fixed;
			cout.precision(9);
			cout << Data[i][j] << "	 ";
		}
		cout << "|" << endl;
	}
	cout << "================================" << endl;
}

//Cross product
cdMatrix cdMatrix::operator |(const cdMatrix &_a) const
{
	if (columns != 1 || _a.columns != 1 || rows != 3 || _a.rows != 3)
	{
		cout << "Check your input matrix, this operation cannot be executed at this time. More coding is needed" << endl;
		exit(0);
	}

	cdMatrix result(rows, columns);
	result.Data = (float**) new float*[result.rows];

	for (int i = 0; i < result.rows; i++)
		result.Data[i] = (float*) new float[result.columns];

	result.Data[0][0] = (((Data[1][0])*(_a.Data[2][0])) - ((Data[2][0])*(_a.Data[1][0])));
	result.Data[1][0] = (((Data[2][0])*(_a.Data[0][0])) - ((Data[0][0])*(_a.Data[2][0])));
	result.Data[2][0] = (((Data[0][0])*(_a.Data[1][0])) - ((Data[1][0])*(_a.Data[0][0])));

	return result;
}

void cdMatrix::ComputeTransfromMat4x4(float th, float al, float d, float a)
{
	//Transform matrix
	float A[4][4] =
	{ c(th),    -c(al)*s(th),   s(al)*s(th),    c(th)*a,
		s(th),    c(al)*c(th),    -s(al)*c(th),   s(th)*a,
		0.0,        s(al),          c(al),          d,
		0.0,        0.0,              0.0,              1.0 };

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = A[i][j];
		}
	}
}

void cdMatrix::ComputeRotMat3x3(float th, float al, float d, float a)
{
	//Transform matrix
	float A[3][3] =
	{ c(th),    -c(al)*s(th),   s(al)*s(th),
		s(th),    c(al)*c(th),    -s(al)*c(th),
		0.0,        s(al),          c(al) };


	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = A[i][j];
		}
	}
}

void cdMatrix::ComputeRotMat3x3(float th, char dir)
{
	if (dir == 'x' || dir == 'X')
	{
		float A[3][3] =
		{
			1.0,  0.0,      0.0,
			0.0,  c(th),  -s(th),
			0.0,  s(th),  c(th)
		};
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				Data[i][j] = A[i][j];
			}
		}

	}
	if (dir == 'y' || dir == 'Y')
	{
		float A[3][3] =
		{
			c(th),  0.0,  s(th),
			0.0,      1.0,  0.0,
			-s(th), 0.0,  c(th)
		};
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				Data[i][j] = A[i][j];
			}
		}
	}
	if (dir == 'z' || dir == 'Z')
	{
		float A[3][3] =
		{
			c(th),  -s(th), 0.0,
			s(th),  c(th),  0.0,
			0.0,      0.0,      1.0
		};
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				Data[i][j] = A[i][j];
			}
		}
	}
}

void cdMatrix::ComputeRotMat4x4(float th, char dir)
{
	if (dir == 'x' || dir == 'X')
	{
		float A[4][4] =
		{
			1.0,  0.0,      0.0,         0.0,
			0.0,  c(th),  -s(th),    0.0,
			0.0,  s(th),  c(th),     0.0,
			0.0,  0.0,      0.0,         1.0
		};
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				Data[i][j] = A[i][j];
			}
		}

	}
	if (dir == 'y' || dir == 'Y')
	{
		float A[4][4] =
		{
			c(th),  0.0,  s(th),  0.0,
			0.0,      1.0,  0.0,      0.0,
			-s(th), 0.0,  c(th),  0.0,
			0.0,      0.0,  0.0,      1.0
		};
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				Data[i][j] = A[i][j];
			}
		}
	}
	if (dir == 'z' || dir == 'Z')
	{
		float A[4][4] =
		{
			c(th),  -s(th), 0.0,  0.0,
			s(th),  c(th),  0.0,  0.0,
			0.0,      0.0,      1.0,  0.0,
			0.0,      0.0,      0.0,  1.0
		};
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				Data[i][j] = A[i][j];
			}
		}
	}
}

void cdMatrix::ComputeTransMat4x4(float dx, float dy, float dz)
{
	float A[4][4] =
	{
		1.0,  0.0,  0.0,  dx,
		0.0,  1.0,  0.0,  dy,
		0.0,  0.0,  1.0,  dz,
		0.0,  0.0,  0.0,  1.0
	};

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = A[i][j];
		}
	}
}

void cdMatrix::ComputeTrace3x3(float x, float y, float z)
{
	float A[3][3] =
	{
		x, 0.0, 0.0,
		0.0, y, 0.0,
		0.0, 0.0, z
	};
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = A[i][j];
		}
	}
}

void cdMatrix::ComputeIdentity3x3()
{
	float A[3][3] =
	{
		1.0,  0.0,  0.0,
		0.0,  1.0,  0.0,
		0.0,  0.0,  1.0
	};
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			Data[i][j] = A[i][j];
		}
	}
}

void cdMatrix::Norm() // only vector is
{
	if (columns != 1)
	{
		cout << "Check your input matrix, this operation cannot be executed at this time. More coding is needed" << endl;
		exit(0);
	}
	float length = sqrt(Data[0][0] * Data[0][0] + Data[1][0] * Data[1][0] + Data[2][0] * Data[2][0]);
	Data[0][0] = Data[0][0] / length;
	Data[1][0] = Data[1][0] / length;
	Data[2][0] = Data[2][0] / length;
}

float cdMatrix::Length()
{
	if (columns != 1)
	{
		cout << "Check your input matrix, this operation cannot be executed at this time. More coding is needed" << endl;
		exit(0);
	}

	float length = sqrt(Data[0][0] * Data[0][0] + Data[1][0] * Data[1][0] + Data[2][0] * Data[2][0]);
	return length;
}