/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

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
