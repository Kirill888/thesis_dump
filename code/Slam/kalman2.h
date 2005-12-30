#ifndef __KALMAN2_H__
#define __KALMAN2_H__
#include "geometry.h"

void kalman_update(Gaussian2d *x, const Gaussian2d *obs, int nobs);
void kalman_update(Gaussian2d *x, const Gaussian2d *obs);

void kalman_update(RobotPoseCov *x, const RobotPoseCov *obs);

//Incorporate range/bearing observation
void ekf_update(Gaussian2d *x, double range, double bearing, 
                const double Szz[2][2], const RobotPose *robot);

void ekf_update(Gaussian2d *x, double range, double bearing, 
                const double Szz[2][2], const RobotPose *robot,
		const RobotPose *sensor);
#endif
