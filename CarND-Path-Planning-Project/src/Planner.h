#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "Executor.h"

using namespace std;

class Planner {

public:

  Executor exector;

  /* Constructo  */
  Planner();

  /* Destructor */
  virtual ~Planner();

  /* Initialize */
  void Init(Executor& e);

  /* Update the PID error variables given cross track error */
  void Update();

  double Cost();

};

#endif /* PLANNER_H */
