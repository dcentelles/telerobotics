/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <ROVOperator.h>

namespace dcauv {

ROVOperator::ROVOperator() {
	// TODO Auto-generated constructor stub
}

ROVOperator::~ROVOperator() {
	// TODO Auto-generated destructor stub
}

void ROVOperator::SetDesiredState(void * _data, unsigned int _length)
{

}

void ROVOperator::SetImageReceivedCallback(std::function<void(void*, unsigned int)> _callback)
{
	imageReceivedCallback = _callback;
}

void ROVOperator::SetStateReceivedCallback(std::function<void(void*, unsigned int)> _callback)
{
	stateReceivedCallback = _callback;
}

void ROVOperator::Start()
{
}

} /* namespace dcauv */
