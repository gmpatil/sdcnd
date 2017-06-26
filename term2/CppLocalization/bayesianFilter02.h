//============================================================================
// Name        : bayesianFilter.h
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#ifndef BAYESIANFILTER_H_
#define BAYESIANFILTER_H_

#include <vector>
#include <string>
#include <fstream>

#include "measurement_package01.h"
#include "map01.h"
#include "help_functions01.h"

class bayesianFilter {
public:
  
  bool is_initialized_;
  
	//constructor:
	bayesianFilter();
	//deconstructor:
	virtual ~bayesianFilter();


	//main public member function, which estimate the beliefs:
	void process_measurement(const MeasurementPackage &measurements,
			                 const map &map_1d,
			                 help_functions &helpers);

	//member public: belief of state x:
	std::vector<float> bel_x ;

private:

/////private members:

	////////////////
    //Add members!//
    ////////////////

  double control_std_;
  std::vector<float> bel_x_init;
  void belInit(double loc, help_functions &helpers);
  
};

#endif /* BAYESIANFILTER_H_ */
