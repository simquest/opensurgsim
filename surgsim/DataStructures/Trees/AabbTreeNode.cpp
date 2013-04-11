#ifndef __AABB_h__
#define __AABB_h__

#include <float.h> // DBL_MAX, DBL_MIN
//#include <GL/gl.h>

class _AABB_
{
public: // Only for debug purposes !
  double min[3];
  double max[3];
  double *bound[2]; // min,max respectively indexed 0,1
protected:

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

public:
  _AABB_(){ bound[0]=min; bound[1]=max; };
  _AABB_(const _AABB_ &original)
  {
    min[0]=original.min[0];  min[1]=original.min[1];  min[2]=original.min[2];
    max[0]=original.max[0];  max[1]=original.max[1];  max[2]=original.max[2];
    bound[0]=min; bound[1]=max;
  };
  _AABB_& operator =(const _AABB_ &original)
  {
    min[0]=original.min[0];  min[1]=original.min[1];  min[2]=original.min[2];
    max[0]=original.max[0];  max[1]=original.max[1];  max[2]=original.max[2];
    bound[0]=min; bound[1]=max;
    return *this;
  };
  _AABB_(double *_min,double *_max)
  {
    min[0]=_min[0];  min[1]=_min[1];  min[2]=_min[2];
    max[0]=_max[0];  max[1]=_max[1];  max[2]=_max[2];
    bound[0]=min; bound[1]=max;
  };
  void init(void){ bound[0]=min; bound[1]=max; }; // Need to be called if _AABB_ allocated with a simple malloc !

  void reset(void)
  {
    min[0]=DBL_MAX; max[0]=-DBL_MAX;
    min[1]=DBL_MAX; max[1]=-DBL_MAX;
    min[2]=DBL_MAX; max[2]=-DBL_MAX;
  };

  void addPoint(double x,double y,double z)
  {
    if(x<min[0]) min[0]=x;
    if(x>max[0]) max[0]=x;

    if(y<min[1]) min[1]=y;
    if(y>max[1]) max[1]=y;

    if(z<min[2]) min[2]=z;
    if(z>max[2]) max[2]=z;
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

  void expand(const _AABB_ &box)
  {
	  if(box.min[0]<min[0]) min[0]=box.min[0];
	  if(box.max[0]>max[0]) max[0]=box.max[0];

	  if(box.min[1]<min[1]) min[1]=box.min[1];
	  if(box.max[1]>max[1]) max[1]=box.max[1];

	  if(box.min[2]<min[2]) min[2]=box.min[2];
	  if(box.max[2]>max[2]) max[2]=box.max[2];
  };

  //void draw(bool wireframe=true)
  //{
  //  if(wireframe)
  //  {
  //    glBegin(GL_LINE_STRIP);
  //    // Top (Y direction)
  //    glVertex3d( min[0] , max[1] , min[2] );
  //    glVertex3d( max[0] , max[1] , min[2] );
  //    glVertex3d( max[0] , max[1] , max[2] );
  //    glVertex3d( min[0] , max[1] , max[2] );
  //    glVertex3d( min[0] , max[1] , min[2] );
  //    glEnd();

  //    glBegin(GL_LINE_STRIP);
  //    // Bottom (Y direction)
  //    glVertex3d( min[0] , min[1] , min[2] );
  //    glVertex3d( max[0] , min[1] , min[2] );
  //    glVertex3d( max[0] , min[1] , max[2] );
  //    glVertex3d( min[0] , min[1] , max[2] );
  //    glVertex3d( min[0] , min[1] , min[2] );
  //    glEnd();

  //    glBegin(GL_LINES);
  //    // 4 Edges along Y direction
  //    glVertex3d( min[0] , min[1] , min[2] ); glVertex3d( min[0] , max[1] , min[2] );
  //    glVertex3d( max[0] , min[1] , min[2] ); glVertex3d( max[0] , max[1] , min[2] );
  //    glVertex3d( max[0] , min[1] , max[2] ); glVertex3d( max[0] , max[1] , max[2] );
  //    glVertex3d( min[0] , min[1] , max[2] ); glVertex3d( min[0] , max[1] , max[2] );
  //    glEnd();
  //  }else{
  //    glBegin(GL_TRIANGLE_STRIP);
  //    glNormal3f(0,0,-1);
  //    glVertex3d( min[0] , min[1] , min[2] ); // Face Zmin
  //    glVertex3d( min[0] , max[1] , min[2] ); // Face Zmin

  //    glVertex3d( max[0] , min[1] , min[2] ); // Face Zmin
  //    glVertex3d( max[0] , max[1] , min[2] ); // Face Zmin

  //    glNormal3f(1,0,0);
  //    glVertex3d( max[0] , min[1] , max[2] ); // Face Xmax
  //    glVertex3d( max[0] , max[1] , max[2] ); // Face Xmax

  //    glNormal3f(0,0,1);
  //    glVertex3d( min[0] , min[1] , max[2] ); // Face Zmax
  //    glVertex3d( min[0] , max[1] , max[2] ); // Face Zmax

  //    glNormal3f(-1,0,0);
  //    glVertex3d( min[0] , min[1] , min[2] ); // Face Xmin
  //    glVertex3d( min[0] , max[1] , min[2] ); // Face Xmin
  //    glEnd();

  //    glBegin(GL_TRIANGLE_STRIP);
  //    // Face Ymin
  //    glNormal3f(0,-1,0);
  //    glVertex3d( min[0] , min[1] , min[2] );
  //    glVertex3d( min[0] , min[1] , max[2] );
  //    glVertex3d( max[0] , min[1] , min[2] );
  //    glVertex3d( max[0] , min[1] , max[2] );
  //    glEnd();

  //    glBegin(GL_TRIANGLE_STRIP);
  //    // Face Ymax
  //    glNormal3f(0,1,0);
  //    glVertex3d( min[0] , max[1] , min[2] );
  //    glVertex3d( min[0] , max[1] , max[2] );
  //    glVertex3d( max[0] , max[1] , min[2] );
  //    glVertex3d( max[0] , max[1] , max[2] );
  //    glEnd();
  //  }
  //};
};

#endif
