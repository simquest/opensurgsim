template <class Matrix, class Vector> bool LCP<Matrix,Vector>::
isLCPsolved(IN int n, IN int m, IN Matrix& A, IN Vector& b, IN Vector& x, OUT Vector& c)
{

	//! Compute c = Ax+b.
	for (int i=0 ; i<n ; i++)
	{
		c[i] = b[i];
		for (int j=0 ; j<m ; j++)
		{
			c[i] += A[i][j]*x[j];
		}
	}

	//! Check if we have: 0 <= x orthogonal c => 0
	for (int i=0 ; i<n ; i++)
		if (x[i]<0 || c[i]<0 || x[i]*c[i]!=0)
		{
			return false;
		}

	return true;
}
