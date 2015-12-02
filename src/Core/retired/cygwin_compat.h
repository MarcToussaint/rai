/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#ifndef __MT_CYGWIN_COMPAT_HPP
#define __MT_CYGWIN_COMPAT_HPP

#ifdef __CYGWIN__

// the syscall is only used for determining the thread id, which we only need
// for debugging, fortunately, so we can hack around it 
int syscall(int, ...) {
	return 1;
}
#define SYS_gettid 0

// some differences on pthread
int pthread_setname_np(pthread_t, const char*) { 
  return 0; 
}
#define PTHREAD_MUTEX_RECURSIVE_NP PTHREAD_MUTEX_RECURSIVE

#endif // __CYGWIN__

#endif // __MT_CYGWIN_COMPAT_HPP
