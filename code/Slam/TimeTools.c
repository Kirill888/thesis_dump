/**
 * TimeTools.c - simple stuff for date/time
 * 
 * Copyright (C) 2000,2001 David Austin
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
 *   $Id: TimeTools.c,v 1.2 2001/09/14 23:01:18 david Exp $ 
 */

#ifndef DEPEND
#include <sys/time.h>
#include <unistd.h>
#endif /* DEPEND */

#include "TimeTools.h"

/*===========================================================================*/

double GetCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ((double) tv.tv_sec) + 1.0e-6 * ((double) tv.tv_usec);
}

/*===========================================================================*/

/*
 * Local Variables:
 * mode: C
 * End:
 */
