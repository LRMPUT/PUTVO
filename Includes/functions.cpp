#include "includes.h"


// XYZ convention
void TransformationToEuler(Eigen::Matrix4f transformation, float *Orient)
{
	if (abs(transformation(2, 2)) < std::numeric_limits<double>::epsilon()
			&& abs(transformation(1, 2))
					< std::numeric_limits<double>::epsilon()) {
		Orient[0] = 0;
		Orient[1] = atan2(transformation(0, 2), transformation(2, 2))
						* 180/M_PI;
		Orient[2] = atan2(transformation(1, 0), transformation(1, 1))
								* 180/M_PI;
	}
	else 
	{
		Orient[0] = atan2(-transformation(1,2), transformation(2,2));
		double sr = sin (Orient[0]);
		double cr = cos (Orient[0]);
		Orient[1] = atan2(transformation(0,2),cr * transformation(2,2) - sr * transformation(1,2))*180/M_PI;
		Orient[2] = atan2(-transformation(0,1),transformation(0,0))*180/M_PI;
		Orient[0] *= 180.0/M_PI;
	}
}

void calculateTransformation(Eigen::Matrix4f &transformation,float alpha, float beta, float gamma, float tx, float ty, float tz)
{

    float s1=sin(alpha);
    float s2=sin(beta);
    float s3=sin(gamma);
    float c1=cos(alpha);
    float c2=cos(beta);
    float c3=cos(gamma);

    float r11=c2; float r12=-c3*s2; float r13=s2*s3;
    float r21=c1*s2; float r22=c1*c2*c3-s1*s3; float r23=-c3*s1-c1*c2*s3;
    float r31=s1*s2; float r32=c1*s3+c2*c3*s1; float r33=c1*c3-c2*s1*s3;
    
    r11 =  1;
    r12 = r13 = 0;
    r21 = 0;
    r22 = c1;
    r23 = -s1;
    r31 = 0;
    r32 = s1;
    r33 = c1;

	transformation = Eigen::Matrix4f::Identity();

    transformation(0,0)=r11;
    transformation(0,1)=r12;
    transformation(0,2)=r13;

    transformation(1,0)=r21;
    transformation(1,1)=r22;
    transformation(1,2)=r23;

    transformation(2,0)=r31;
    transformation(2,1)=r32;
    transformation(2,2)=r33;

    transformation(0,3)=tx;
    transformation(1,3)=ty;
    transformation(2,3)=tz;
}
