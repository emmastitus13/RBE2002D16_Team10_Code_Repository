#ifndef TURN_H
#define TURN_H

#include "DriveTrain.h"
#include "definitions.h"

extern unsigned long curRTicks;
extern unsigned long curLTicks;
extern unsigned int tickRDiff;
extern unsigned int tickLDiff;
extern volatile unsigned long lEncode;
extern volatile unsigned long rEncode;
extern const  int tickPer90;

class Turn{
	public:
		Turn(DriveTrain& rb);
		bool turnSlightLeft(void);
		bool turnSlightRight(void);

	private:
		DriveTrain& robotDrive;
};


#endif
