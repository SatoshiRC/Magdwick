/*
 * Magdwick.h
 *
 *  Created on: Feb 11, 2023
 *      Author: OHYA Satoshi
 */

#ifndef MAGDWICK_MADGWICK_H_
#define MAGDWICK_MADGWICK_H_

#include <cmath>
#include <array>
#include "../Quaternion/Quaternion.h"
#include <stdint.h>

template<typename T>
class Madgwick {
public:
	Madgwick(){};

	/* brief calculate quaternion
	 * param accelValue Value of accele meter (m/s^2)
	 * param gyroValue Value of gyroscope (rad/s)
	 * param time Time since start program (ms)
	 */
	void update(std::array<T, 3> accelValue, std::array<T, 3> gyroValue, T time);

	/* brief gets an attitude in quaternion
	 * return quaternion
	 */
	Quaternion<T> getQuaternion(){return quaternion;}
private:
	Quaternion<T> quaternion;
	T befTime;
};

#endif /* MAGDWICK_MADGWICK_H_ */
