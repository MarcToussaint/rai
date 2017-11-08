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


#include "cma.h"
#include "algos.h"




CMA::CMA() {




}
CMA::~CMA() {

}

void CMA::run(){
    //initialize
    arr pc,ps;
    m.resize(dim);
    pc.resize(dim);
    ps.resize(dim);
    m.setZero();
    pc.setZero();
    ps.setZero();
    C = eye(dim);


}

