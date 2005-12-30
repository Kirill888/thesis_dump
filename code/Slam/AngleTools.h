/**
 * AngleTools.h - tools for manipulation of angles
 * 
 * Copyright (C) 1999 Patric Jensfelt
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 * 
 * 
 * Longer description
 * 
 * 
 * @author  David Austin
 *          d.austin@computer.org
 *          Dept of Systems Engineering,
 *          Research School of Information Sciences and Engineering,
 *          Australian National University,
 *          Canberra, 0200, Australia.
 *
 * @version 
 *   $Id: AngleTools.h,v 1.2 2002/05/21 03:34:27 david Exp $ 
 */

#ifndef __INC_AngleTools_h
#define __INC_AngleTools_h

#ifndef DEPEND
#include <math.h>
#endif  // DEPEND

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Defines function used in the context of angle, convertions etc
 *
 * To use them:
 * #include "AngleTools.h"
 * 
 * @author Patric Jensfelt
 * @see DistanceTools
 */

  /** Transforms angle in degs to radians */
  double deg2rad(const double ang);

  /** Transforms angle in degs to milli radians */
  double deg2mrad(const double ang);

  /** Transforms angle in tens of degs to radians */
  double tdeg2rad(const double ang);

  /** Transforms angle in tens of degs to milli radians */
  double tdeg2mrad(const double ang);

  /** Transforms angle in radians to degs */
  double rad2deg(const double ang);

  /** Transforms angle in radians to tens of degs */
  long rad2tdeg(const double ang);

  /** Transforms angle in milli radians to degs */
  double mrad2deg(const double ang);

  /** Transforms angle in milli radians to tens of degs */
  long mrad2tdeg(const double ang);

  /** Returns the angle modulated into [0, 360) */
  double mod360(const double ang);

  /** Returns the angle modulated into [-180, 180) */
  double mod180(const double ang);

  /** Returns the angle modulated into [0, 3600) */
  long mod3600(const double ang);

  /** Returns the angle in (-1800,1800) */
  long mod1800(const double ang);

  /** Return the angle [-pi, pi) */
  double modPI(const double ang);

  /** Return the angle [0, 2*pi) */
  double mod2PI(const double ang);

  /**
   * Returns the angular difference between angle a1 and angle a2, (a1
   * - a2) where these angles are expressed in degrees. The result is
   * given in the interval (-180, 180].
   *
   * &param a1 The first angle in degs 
   * &param a2 The second angle in degs 
   */
  double angleDiffDeg(const double a1, const double a2);
 
  /**
   * Returns the angular difference between angle a1 and angle a2, (a1
   * - a2) where these angles are expressed in tens of degrees. The
   * result is given in the interval (-1800, 1800].
   *
   * &param a1 The first angle in tens of degs 
   * &param a2 The second angle in tens of degs */
  long angleDiffTDeg(const double a1, const double a2);

  /**
   * Returns the angular difference between angle a1 and angle a2, (a1
   * - a2) where these angles are expressed in radians. The result is
   * given in the interval (-pi, pi].  
   *
   * &param a1 The first angle in rads 
   * &param a2 The second angle in rads 
   */
  double angleDiffRad(const double a1, const double a2);

inline double 
deg2rad(const double ang) 
{ return M_PI / 180.0 * ang; }

inline double 
deg2mrad(const double ang)
{ return M_PI / 0.18 * ang; }

inline double 
tdeg2rad(const double ang)
{ return M_PI / 1800.0 * ang; }

inline double 
tdeg2mrad(const double ang)
{ return M_PI / 1.8 * ang; }

inline double 
rad2deg(const double ang)
{ return 180.0 / M_PI * ang; }

inline long 
rad2tdeg(const double ang)
{ return long(1800.0 / M_PI * ang); }

inline double 
mrad2deg(const double ang)
{ return 0.18 / M_PI * ang; }

inline long 
mrad2tdeg(const double ang)
{ return long(1.8 / M_PI * ang); }

inline double 
mod360(const double ang)
{ 
  double tmp = ang;
  
  // Make sure that it is larger than or equal to 0 degs
  while (tmp < 0)
    tmp += 360.0;
  
  // Make sure that it is less than 360 degs
  while (tmp > 360)
    tmp -= 360.0;
  
  return tmp;
}

inline double 
mod180(const double ang)
{ 
  double tmp = ang;
  
  // Make sure that it is larger than or equal to -180 degs
  while (tmp < -180)
    tmp += 360.0;
  
  // Make sure that it is less than 180 degs
  while (tmp > 180)
    tmp -= 360.0;
  
  return tmp;
}

inline long 
mod3600(const double ang)
{ 
  double tmp = ang;
  
  // Make sure that it is larger than or equal to 0 degs
  while (tmp < 0)
    tmp += 3600.0;
  
  // Make sure that it is less than 3600 tens of degs
  while (tmp > 3600)
    tmp -= 3600.0;
  
  return long(tmp);
}

inline long 
mod1800(const double ang)
{ 
  double tmp = ang;
  
  // Make sure that it is larger than or equal to -1800 degs
  while (tmp < -1800)
    tmp += 3600.0;
  
  // Make sure that it is less than 1800 tens of degs
  while (tmp > 1800)
    tmp -= 3600.0;
  
  return long(tmp);
}

inline double 
modPI(const double ang)
{ 
  double tmp = ang;
  const double TWOPI = 2.0 * M_PI;
  
  // Make sure that it is larger than or equal to -pi 
  while (tmp < -M_PI)
    tmp += TWOPI;
  
  // Make sure that it is less than pi
  while (tmp > M_PI)
    tmp -= TWOPI;
  
  return tmp;
}

inline double 
mod2PI(const double ang)
{
  double TWOPI = 2.0 * M_PI;
  
  double tmp = ang;
  
  // Make sure that it is larger than or equal to 0
  while(tmp < 0)
    tmp += TWOPI;
  
  // Make sure that it is less than 2*M_PI
  while(tmp > TWOPI)
    tmp -= TWOPI;
  
  return tmp;
}

inline double 
angleDiffDeg(const double a1, const double a2)
{
  double tmp = a1 - a2;
  
  while (tmp < -180.0)
    tmp += 360.0;
  
  while (tmp > 180.0)
    tmp -= 360.0;
  
  return tmp;
}

inline long 
angleDiffTDeg(const double a1, const double a2)
{
  double tmp = a1 - a2;
  
  while (tmp < -1800.0)
    tmp += 3600.0;
  
  while (tmp > 1800.0)
    tmp -= 3600.0;
  
  return long(tmp);
}

inline double 
angleDiffRad(const double a1, const double a2)
{
  const double TWOPI = 2.0 * M_PI;
  
  double tmp = a1 - a2;
  
  while (tmp < -M_PI)
    tmp += TWOPI;
  
  while (tmp > M_PI)
    tmp -= TWOPI;
  
  return tmp;
}

#endif /* __INC_AngleTools_h */

/*
 * Local Variables:
 * mode: C++
 * End:
 */
