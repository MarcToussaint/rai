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
#include <unordered_set>
#include "filter.h"

Filter::Filter()
  : Thread("filter", -1),
    percepts_input(this, "percepts_input", true), //listens!!
    percepts_filtered(this, "percepts_filtered", false)
{
  threadOpen();
}

Filter::~Filter(){
  threadClose();
}

void Filter::step(){

  if(step_count<10){
    percepts_input.writeAccess();
    percepts_filtered.writeAccess();

    listDelete(percepts_filtered());
    percepts_filtered() = percepts_input();
    percepts_input().clear();

    percepts_filtered.deAccess();
    percepts_input.deAccess();

    return;
  }

  percepts_input.writeAccess();
  percepts_filtered.writeAccess();

  if(verbose>0) cout <<"FILTER: #inputs=" <<percepts_input().N <<" #database=" <<percepts_filtered().N <<endl;
//  if(verbose>1){
//    for(Percept *p:inputs) cout <<"  in: " <<(*p) <<endl;
////    for(Percept *p:inputs) LOG(0) <<"  in: " <<*p;
//  }

  // If empty inputs, do nothing.
  if (percepts_input().N == 0 && percepts_input().N == 0) {
    percepts_filtered.deAccess();
    percepts_input.deAccess();
    return;
  }

  PerceptL filtered;

  // For each type of inputs, run the algorithm.
  for (auto const& type : {Percept::Type::alvar,
                           Percept::Type::cluster,
                           Percept::Type::plane,
                           Percept::Type::optitrackbody,
                           Percept::Type::optitrackmarker})
  {
    // Grab the subset from the inputs matching this type
    PerceptL input_ofType;
    for (Percept *p : percepts_input()) {
      if (p->type == type) {
        input_ofType.append(p);
      }
    }

    // Find all matching object of this type
    PerceptL database_ofType;
    for (Percept *p : percepts_filtered()) {
      if (p->type == type) {
        database_ofType.append(p);
      }
    }

    if (input_ofType.N == 0 && database_ofType.N == 0)
      continue;

    // Create bipartite costs
    costs = createCostMatrix(input_ofType, database_ofType);

    // Run Hungarian algorithm
    Hungarian ha(costs);

    // Now we have the optimal matching. Assign values.
    PerceptL assignedObjects = assign(input_ofType, database_ofType, ha);

    filtered.append(assignedObjects);
  }

  //-- delete objects in percepts and database that are not filtered
  for (Percept* p : percepts_input()) {
    if (!filtered.contains(p))
      delete p;
  }
  percepts_input().clear();

  for (Percept* p : percepts_filtered()) {
    if (!filtered.contains(p))
      delete p;
  }
  percepts_filtered().clear();

  // Set the variable
  percepts_filtered() = filtered;
  percepts_filtered.deAccess();
  percepts_input.deAccess();
}

PerceptL Filter::assign(const PerceptL& input, const PerceptL& database, const Hungarian& ha)
{
  PerceptL new_objects;
  std::unordered_set<int> matched_ids;

  uint num_old = database.N;
  uint num_new = input.N;

  for (uint i = 0; i < ha.starred.dim(0); ++i ) {
    uint col = ha.starred[i]().maxIndex();
    // 3 cases:
    // 1) Existed before, no longer exists. If i > num_new
    // 2) Existed before and still exists. If costs < distannce_threshold
    // 3) Didn't exist before. This happens iff col >= num_old

    // Existed before, doesn't exist now.
    if ( i >= num_new ) {
      //std::cout<< "Existed before, doesn't now." << std::endl;
      Percept *new_obj = database(col);
      new_obj->relevance *= relevance_decay_factor;
      new_objects.append(new_obj);
    } else {
      if ( ( col < num_old ) && (costs(i, col) < distance_threshold) ) { // Existed before
        //std::cout<< "Existed before, does now" << std::endl;
        Percept *new_obj = input(i);
        if (new_obj->type != Percept::Type::alvar)
          new_obj->id = database(col)->id;
        new_objects.append( new_obj );
      } else { // This didn't exist before. Add it in
        //std::cout<< "Didn't exist before, or not close enough." << std::endl;
        Percept *new_obj = input(i);
        if (new_obj->type != Percept::Type::alvar) {
          new_obj->id = maxId;
          maxId++;
        }
        new_objects.append(new_obj);
        //std::cout << "Didn't exist before. Col >= num_old: " << col << ' ' << num_old << std::endl;
      }
    }
    matched_ids.insert(new_objects(i)->id);
    //std::cout << "Assigning \t" << i << "\tMatches:\t" << matched_ids(i).id << "\t Relevance: " << perceps(i).relevance << std::endl;
  }

  // For each of the old objects, update the relevance factor.
  for ( uint i = 0; i < database.N; ++i ) {
    if ( matched_ids.find(database(i)->id) == matched_ids.end() ) {
      Percept *new_obj = database(i);
      new_obj->relevance *= relevance_decay_factor;
      new_objects.append(new_obj);      
    }
  }
  PerceptL cleaned;
  uint count = new_objects.N;
  for ( uint i = 0; i < count; ++i ) {
    if(new_objects(i)->relevance > relevance_threshold) {
      cleaned.append(new_objects(i));
    }
  }

  return cleaned;
}

arr Filter::createCostMatrix(const PerceptL& newObjects, const PerceptL& oldObjects)
{
  // First, make a padded out matrix to ensure it is bipartite.
  uint num_new = newObjects.N;
  uint num_old = oldObjects.N;
  uint dims = std::max(num_old, num_new);
  arr costs = ones(dims, dims) * -1.0;

  // Assign costs
  for (uint i=0; i<num_new; ++i) for (uint j=0; j<num_old; ++j) {
    costs(i,j) = newObjects(i)->idMatchingCost(*oldObjects(j));
  }

  // For every element that hasn't been set, set the costs to the max.
  double max_costs = costs.max();

  for (uint i=0; i<dims; ++i) for (uint j=0; j<dims; ++j) {
    if (( i >= num_new ) || ( j >= num_old ))
      costs(i,j) = max_costs;
  }
  return costs;
}
