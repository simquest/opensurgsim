bool broadPhase_isIntersecting(double *A, double *AB,double *inv_AB,int *ABsign,double &tmin,double &tmax)
{
	double txyzmin,txyzmax,XYZ;

	tmin= DBL_MAX;
	tmax=-DBL_MAX;

	// Test X plane, min direction
	txyzmin = (bound[  ABsign[0]][0]-A[0])*inv_AB[0]; // Can only be an entry point
	//if(txyzmin>=0.0 && txyzmin<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[1] + txyzmin*AB[1];
		if(XYZ>=min[1] && XYZ<=max[1]) // entry point on the face !? Test Y axis...
		{
			XYZ = A[2] + txyzmin*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // entry point on the face !? Test Z axis...
				if(txyzmin<tmin) // We have a collision on the face => THIS IS AN ENTRY POINT !!
					tmin = txyzmin; // Keep the earliest entry point !
		}
	}
	// Test X plane, max direction
	txyzmax = (bound[1-ABsign[0]][0]-A[0])*inv_AB[0]; // Can only be an exit  point
	//if(txyzmax>=0.0 && txyzmax<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[1] + txyzmax*AB[1];
		if(XYZ>=min[1] && XYZ<=max[1]) // exit point on the face !? Test Y axis...
		{
			XYZ = A[2] + txyzmax*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // exit point on the face !? Test Z axis...
				if(txyzmax>tmax) // We have a collision on the face => THIS IS AN EXIT POINT !!
					tmax = txyzmax; // Keep the latest entry point !
		}
	}

	// Test Y plane, min direction
	txyzmin = (bound[  ABsign[1]][1]-A[1])*inv_AB[1]; // Can only be an entry point
	//if(txyzmin>=0.0 && txyzmin<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmin*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // entry point on the face !? Test X axis...
		{
			XYZ = A[2] + txyzmin*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // entry point on the face !? Test Z axis...
				if(txyzmin<tmin) // We have a collision on the face => THIS IS AN ENTRY POINT !!
					tmin = txyzmin; // Keep the earliest entry point !
		}
	}
	// Test Y plane, max direction
	txyzmax = (bound[1-ABsign[1]][1]-A[1])*inv_AB[1]; // Can only be an exit  point
	//if(txyzmax>=0.0 && txyzmax<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmax*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // exit point on the face !? Test X axis...
		{
			XYZ = A[2] + txyzmax*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // exit point on the face !? Test Z axis...
				if(txyzmax>tmax) // We have a collision on the face => THIS IS AN EXIT POINT !!
					tmax = txyzmax; // Keep the latest exit point !
		}
	}

	// Test Z plane, min direction
	txyzmin = (bound[  ABsign[2]][2]-A[2])*inv_AB[2]; // Can only be an entry point
	//if(txyzmin>=0.0 && txyzmin<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmin*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // entry point on the face !? Test X axis...
		{
			XYZ = A[1] + txyzmin*AB[1];
			if(XYZ>=min[1] && XYZ<=max[1]) // entry point on the face !? Test Y axis...
				if(txyzmin<tmin) // We have a collision on the face => THIS IS AN ENTRY POINT !!
					tmin = txyzmin; // Keep the earliest exit point !
		}
	}
	// Test Z plane, max direction
	txyzmax = (bound[1-ABsign[2]][2]-A[2])*inv_AB[2]; // Can only be an exit  point
	//if(txyzmax>=0.0 && txyzmax<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmax*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // exit point on the face !? Test X axis...
		{
			XYZ = A[1] + txyzmax*AB[1];
			if(XYZ>=min[1] && XYZ<=max[1]) // exit point on the face !? Test Y axis...
				if(txyzmax>tmax) // We have a collision on the face => THIS IS AN EXIT POINT !!
					tmax = txyzmax; // Keep the latest exit point !
		}
	}

	bool enter = (tmin>=0.0 && tmin<=1.0);
	bool exit  = (tmax>=0.0 && tmax<=1.0);
	bool _1stExtremityIsInside = (tmin<0.0 && tmax>0.0);
	bool _2ndExtremityIsInside = (tmin<1.0 && tmax>1.0);

	return (enter || exit || _1stExtremityIsInside || _2ndExtremityIsInside);
};


bool broadPhase_isInside(double *p)
{
	if(
		p[0]>=min[0] && p[0]<=max[0] &&
		p[1]>=min[1] && p[1]<=max[1] &&
		p[2]>=min[2] && p[2]<=max[2]
	) return true;
	return false;
};

bool broadPhase_isIntersecting(double *A, double *B,double &tmin,double &tmax)
{
	double AB[3]={ B[0]-A[0] , B[1]-A[1] , B[2]-A[2] };
	double inv_AB[3]={ AB[0]==0.0?DBL_MAX:1.0/AB[0] , AB[1]==0.0?DBL_MAX:1.0/AB[1] , AB[2]==0.0?DBL_MAX:1.0/AB[2] };
	int ABsign[3]={ AB[0]>=0?0:1 , AB[1]>=0?0:1 , AB[2]>=0?0:1 };

	tmin= DBL_MAX;
	tmax=-DBL_MAX;

	double txyzmin,txyzmax,XYZ;

	// Test X plane, min direction
	txyzmin = (bound[  ABsign[0]][0]-A[0])*inv_AB[0]; // Can only be an entry point
	//if(txyzmin>=0.0 && txyzmin<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[1] + txyzmin*AB[1];
		if(XYZ>=min[1] && XYZ<=max[1]) // entry point on the face !? Test Y axis...
		{
			XYZ = A[2] + txyzmin*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // entry point on the face !? Test Z axis...
				if(txyzmin<tmin) // We have a collision on the face => THIS IS AN ENTRY POINT !!
					tmin = txyzmin; // Keep the earliest entry point !
		}
	}
	// Test X plane, max direction
	txyzmax = (bound[1-ABsign[0]][0]-A[0])*inv_AB[0]; // Can only be an exit  point
	//if(txyzmax>=0.0 && txyzmax<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[1] + txyzmax*AB[1];
		if(XYZ>=min[1] && XYZ<=max[1]) // exit point on the face !? Test Y axis...
		{
			XYZ = A[2] + txyzmax*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // exit point on the face !? Test Z axis...
				if(txyzmax>tmax) // We have a collision on the face => THIS IS AN EXIT POINT !!
					tmax = txyzmax; // Keep the latest entry point !
		}
	}

	// Test Y plane, min direction
	txyzmin = (bound[  ABsign[1]][1]-A[1])*inv_AB[1]; // Can only be an entry point
	//if(txyzmin>=0.0 && txyzmin<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmin*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // entry point on the face !? Test X axis...
		{
			XYZ = A[2] + txyzmin*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // entry point on the face !? Test Z axis...
				if(txyzmin<tmin) // We have a collision on the face => THIS IS AN ENTRY POINT !!
					tmin = txyzmin; // Keep the earliest entry point !
		}
	}
	// Test Y plane, max direction
	txyzmax = (bound[1-ABsign[1]][1]-A[1])*inv_AB[1]; // Can only be an exit  point
	//if(txyzmax>=0.0 && txyzmax<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmax*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // exit point on the face !? Test X axis...
		{
			XYZ = A[2] + txyzmax*AB[2];
			if(XYZ>=min[2] && XYZ<=max[2]) // exit point on the face !? Test Z axis...
				if(txyzmax>tmax) // We have a collision on the face => THIS IS AN EXIT POINT !!
					tmax = txyzmax; // Keep the latest exit point !
		}
	}

	// Test Z plane, min direction
	txyzmin = (bound[  ABsign[2]][2]-A[2])*inv_AB[2]; // Can only be an entry point
	//if(txyzmin>=0.0 && txyzmin<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmin*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // entry point on the face !? Test X axis...
		{
			XYZ = A[1] + txyzmin*AB[1];
			if(XYZ>=min[1] && XYZ<=max[1]) // entry point on the face !? Test Y axis...
				if(txyzmin<tmin) // We have a collision on the face => THIS IS AN ENTRY POINT !!
					tmin = txyzmin; // Keep the earliest exit point !
		}
	}
	// Test Z plane, max direction
	txyzmax = (bound[1-ABsign[2]][2]-A[2])*inv_AB[2]; // Can only be an exit  point
	//if(txyzmax>=0.0 && txyzmax<=1.0) // Should not be because the extremities can be fully inside the box => tmin <0 && tmax>1 !!!!
	{
		XYZ = A[0] + txyzmax*AB[0];
		if(XYZ>=min[0] && XYZ<=max[0]) // exit point on the face !? Test X axis...
		{
			XYZ = A[1] + txyzmax*AB[1];
			if(XYZ>=min[1] && XYZ<=max[1]) // exit point on the face !? Test Y axis...
				if(txyzmax>tmax) // We have a collision on the face => THIS IS AN EXIT POINT !!
					tmax = txyzmax; // Keep the latest exit point !
		}
	}

	bool enter = (tmin>=0.0 && tmin<=1.0);
	bool exit  = (tmax>=0.0 && tmax<=1.0);
	bool _1stExtremityIsInside = (tmin<0.0 && tmax>0.0);
	bool _2ndExtremityIsInside = (tmin<1.0 && tmax>1.0);

	return (enter || exit || _1stExtremityIsInside || _2ndExtremityIsInside);
};

bool broadPhase_isIntersecting(const _AABB_ &box, const double tolerance = 0) const
{
	// Look for a separating plane, going through the 6 faces of each AABB
	if( min[0]>=box.max[0]+tolerance || max[0]+tolerance<=box.min[0] ) return false; // The 2 faces along X for each AABB
	if( min[1]>=box.max[1]+tolerance || max[1]+tolerance<=box.min[1] ) return false; // The 2 faces along Y for each AABB
	if( min[2]>=box.max[2]+tolerance || max[2]+tolerance<=box.min[2] ) return false; // The 2 faces along Z for each AABB
	return true;
};