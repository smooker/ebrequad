/**
This code adopted from 'tilt.c' as posted in the original header below.
Changes have been made to make the code to hold the state data in a structure
so that more than one axis can be filtered.
R. Reese 6/2009
**/


/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: tilt.c,v 1.1 2003/07/09 18:23:29 john Exp $
 *
 * 1 dimensional tilt sensor using a dual axis accelerometer
 * and single axis angular rate gyro.  The two sensors are fused
 * via a two state Kalman filter, with one state being the angle
 * and the other state being the gyro bias.
 *
 * Gyro bias is automatically tracked by the filter.  This seems
 * like magic.
 *
 * Please note that there are lots of comments in the functions and
 * in blocks before the functions.  Kalman filtering is an already complex
 * subject, made even more so by extensive hand optimizations to the C code
 * that implements the filter.  I've tried to make an effort of explaining
 * the optimizations, but feel free to send mail to the mailing list,
 * autopilot-devel@lists.sf.net, with questions about this code.
 *
 * 
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 *
 *************
 *
 *  This file is part of the autopilot onboard code package.
 *  
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
//#include <math.h>
#include "Kalman_api.h"


/*
 * Our update rate.  This is how often our state is updated with
 * gyro rate measurements.  For now, we do it every time an
 * 8 bit counter running at CLK/1024 expires.  You will have to
 * change this value if you update at a different rate.
 */

/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect 0.3 rad jitter
 * from the accelerometer.
 */
//static const float	R_angle	= 0.3;
//
static const float	R_angle	= 0.2*57.2958;

/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the acceleromter
 * relative to the gyros.
 */

static const float	Q_angle	= 0.001;
static const float	Q_gyro	= 0.003;


//static const float	Q_angle	= 0.001*57.2958;
//static const float	Q_gyro	= 0.003*57.2958;

void Kalman_InitState(KALDATA* p_kd, float dt)
{
  p_kd->P[0][0] = 1.0;
  p_kd->P[0][1] = 0.0;
  p_kd->P[1][0] = 0.0;
  p_kd->P[1][1] = 1.0;
  p_kd->angle = 0;
  p_kd->q_bias = 0;
  p_kd->rate = 0;
  p_kd->dt = dt;       //delta time
  p_kd->Pdot[0] = 0;
  p_kd->Pdot[1] = 0;
  p_kd->Pdot[2] = 0;
  p_kd->Pdot[3] = 0;
}

/*
 * state_update is called every dt with a biased gyro measurement
 * by the user of the module.  It updates the current angle and
 * rate estimate.
 *
 * The pitch gyro measurement should be scaled into real units, but
 * does not need any bias removal.  The filter will track the bias.
 *
 * Our state vector is:
 *
 *	X = [ angle, gyro_bias ]
 *
 * It runs the state estimation forward via the state functions:
 *
 *	Xdot = [ angle_dot, gyro_bias_dot ]
 *
 *	angle_dot	= gyro - gyro_bias
 *	gyro_bias_dot	= 0
 *
 * And updates the covariance matrix via the function:
 *
 *	Pdot = A*P + P*A' + Q
 *
 * A is the Jacobian of Xdot with respect to the states:
 *
 *	A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
 *	    [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
 *
 *	  = [ 0 -1 ]
 *	    [ 0  0 ]
 *
 * Due to the small CPU available on the microcontroller, we've
 * hand optimized the C code to only compute the terms that are
 * explicitly non-zero, as well as expanded out the matrix math
 * to be done in as few steps as possible.  This does make it harder
 * to read, debug and extend, but also allows us to do this with
 * very little CPU time.
 */
void Kalman_StateUpdate(
    float       q_m, /* Pitch gyro measurement */
    KALDATA*    p_kd
)
{

	float		q, dt;
   
    q = q_m - p_kd->q_bias; 	/* Unbias our gyro */
    dt = p_kd->dt;

	/*
	 * Compute the derivative of the covariance matrix
	 *
	 *	Pdot = A*P + P*A' + Q
	 *
	 * We've hand computed the expansion of A = [ 0 -1, 0 0 ] multiplied
	 * by P and P multiplied by A' = [ 0 0, -1, 0 ].  This is then added
	 * to the diagonal elements of Q, which are Q_angle and Q_gyro.
	 */
	p_kd->Pdot[0] = Q_angle - p_kd->P[0][1] - p_kd->P[1][0];/* 0,0 */	
	p_kd->Pdot[1] = 0 - p_kd->P[1][1];		/* 0,1 */
	p_kd->Pdot[2] = 0 - p_kd->P[1][1];    		/* 1,0 */
	p_kd->Pdot[3] = Q_gyro;			     	/* 1,1 */

	/* Store our unbias gyro estimate */
	p_kd->rate = q;

	/*
	 * Update our angle estimate
	 * angle += angle_dot * dt
	 *       += (gyro - gyro_bias) * dt
	 *       += q * dt
	 */
	p_kd->angle += (q * dt);

	/* Update the covariance matrix */
	p_kd->P[0][0] += (p_kd->Pdot[0] * dt);
	p_kd->P[0][1] += (p_kd->Pdot[1] * dt);
	p_kd->P[1][0] += (p_kd->Pdot[2] * dt);
	p_kd->P[1][1] += (p_kd->Pdot[3] * dt);
}


/*
 * kalman_update is called by a user of the module when a new
 * accelerometer measurement is available.  axy_m and az_m do not
 * need to be scaled into actual units, but must be zeroed and have
 * the same scale.
 *
 * This does not need to be called every time step, but can be if
 * the accelerometer data are available at the same rate as the
 * rate gyro measurement.
 *
 * For a two-axis accelerometer mounted perpendicular to the rotation
 * axis, we can compute the angle for the full 360 degree rotation
 * with no linearization errors by using the arctangent of the two
 * readings.
 *
 * As commented in state_update, the math here is simplified to
 * make it possible to execute on a small microcontroller with no
 * floating point unit.  It will be hard to read the actual code and
 * see what is happening, which is why there is this extensive
 * comment block.
 *
 * The C matrix is a 1x2 (measurements x states) matrix that
 * is the Jacobian matrix of the measurement value with respect
 * to the states.  In this case, C is:
 *
 *	C = [ d(angle_m)/d(angle)  d(angle_m)/d(gyro_bias) ]
 *	  = [ 1 0 ]
 *
 * because the angle measurement directly corresponds to the angle
 * estimate and the angle measurement has no relation to the gyro
 * bias.
 * CHANGED SUCH THAT ANGLE IS CALCULATED EXTERNALLY
 */
void Kalman_Update(
    float       angle_m, /*measured angle*/
    KALDATA*    p_kd
)
{
    float		angle_err;
    float		C_0;
    float		PCt_0;
	float		PCt_1;
    float		E;
    float		K_0, K_1;
    float		t_0;
	float		t_1;
	
	/* Compute our measured angle and the error in our estimate */
	angle_err = angle_m - p_kd->angle;

    p_kd->err = angle_err;
	/*
	 * C_0 shows how the state measurement directly relates to
	 * the state estimate.
 	 *
	 * The C_1 shows that the state measurement does not relate
	 * to the gyro bias estimate.  We don't actually use this, so
	 * we comment it out.
	 */
	C_0 = 1;
	/* const float		C_1 = 0; */

	/*
	 * PCt<2,1> = P<2,2> * C'<2,1>, which we use twice.  This makes
	 * it worthwhile to precompute and store the two values.
	 * Note that C[0,1] = C_1 is zero, so we do not compute that
	 * term.
	 */
	PCt_0 = C_0 * p_kd->P[0][0]; /* + C_1 * P[0][1] = 0 */
	PCt_1 = C_0 * p_kd->P[1][0]; /* + C_1 * P[1][1] = 0 */
		
	/*
	 * Compute the error estimate.  From the Kalman filter paper:
	 * 
	 *	E = C P C' + R
	 * 
	 * Dimensionally,
	 *
	 *	E<1,1> = C<1,2> P<2,2> C'<2,1> + R<1,1>
	 *
	 * Again, note that C_1 is zero, so we do not compute the term.
	 */
	E =	R_angle	+ C_0 * PCt_0;	/*	+ C_1 * PCt_1 = 0 */
	

	/*
	 * Compute the Kalman filter gains.  From the Kalman paper:
	 *
	 *	K = P C' inv(E)
	 *
	 * Dimensionally:
	 *
	 *	K<2,1> = P<2,2> C'<2,1> inv(E)<1,1>
	 *
	 * Luckilly, E is <1,1>, so the inverse of E is just 1/E.
	 */
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
		
	/*
	 * Update covariance matrix.  Again, from the Kalman filter paper:
	 *
	 *	P = P - K C P
	 *
	 * Dimensionally:
	 *
	 *	P<2,2> -= K<2,1> C<1,2> P<2,2>
	 *
	 * We first compute t<1,2> = C P.  Note that:
	 *
	 *	t[0,0] = C[0,0] * P[0,0] + C[0,1] * P[1,0]
	 *
	 * But, since C_1 is zero, we have:
	 *
	 *	t[0,0] = C[0,0] * P[0,0] = PCt[0,0]
	 *
	 * This saves us a floating point multiply.
	 */
	t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] */
	t_1 = C_0 * p_kd->P[0][1]; /* + C_1 * P[1][1]  = 0 */

	p_kd->P[0][0] -= (K_0 * t_0);
	p_kd->P[0][1] -= (K_0 * t_1);
	p_kd->P[1][0] -= (K_1 * t_0);
	p_kd->P[1][1] -= (K_1 * t_1);
	
	/*
	 * Update our state estimate.  Again, from the Kalman paper:
	 *
	 *	X += K * err
	 *
	 * And, dimensionally,
	 *
	 *	X<2> = X<2> + K<2,1> * err<1,1>
	 *
	 * err is a measurement of the difference in the measured state
	 * and the estimate state.  In our case, it is just the difference
	 * between the two accelerometer measured angle and our estimated
	 * angle.
	 */
	p_kd->angle	+= (K_0 * angle_err);
	p_kd->q_bias	+= (K_1 * angle_err);
}
