/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/* basic image I/O */

#ifndef PNM_FILE_H
#define PNM_FILE_H

//#include <cstdlib>
//#include <climits>
//#include <cstring>
#include <fstream>
//#include "image.h"
#include "misc.h"

#define BUF_SIZE 256

namespace felzenszwalb {
template<class T> class image;

class pnm_error { };

void read_packed(unsigned char *data, int size, std::ifstream &f);

void write_packed(unsigned char *data, int size, std::ofstream &f);

/* read PNM field, skipping comments */ 
void pnm_read(std::ifstream &file, char *buf);

image<uchar> *loadPBM(const char *name);
void savePBM(image<uchar> *im, const char *name);

image<uchar> *loadPGM(const char *name);
void savePGM(image<uchar> *im, const char *name);

image<rgb> *loadPPM(const char *name);
void savePPM(image<rgb> *im, const char *name);

template <class T> void load_image(image<T> **im, const char *name);
template <class T> void save_image(image<T> *im, const char *name);
}  // namespace felzenszwalb

#endif
