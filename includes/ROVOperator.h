/*
 * ROVOperator.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef ROVOPERATOR_H_
#define ROVOPERATOR_H_

#include <functional>
#include <iostream>

namespace dcauv {

class ROVOperator {
public:
	ROVOperator();
	virtual ~ROVOperator();
	void SetDesiredState(void * data, unsigned int length);

	//http://stackoverflow.com/questions/2298242/callback-functions-in-c
	/*
	//mode 1:
	typedef void (*f_data)(void*,unsigned int);
	void SetDataReceivedCallback(f_data);
	*/
	//mode 2:
	typedef std::function<void(void*, unsigned int)> f_data;

	void SetImageReceivedCallback(f_data);
	void SetStateReceivedCallback(f_data);

	void Start();
private:
	f_data imageReceivedCallback;
	f_data stateReceivedCallback;
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
