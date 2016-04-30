#ifndef TURN_H
#define TURN_H

extern curRTicks;
extern curLTicks;
extern tickRDiff;
extern tickLDiff;
extern lEncode;
extern rEncode;

class Turn{
	public:
		Turn(DriveTrain& rb);
		bool turnSlightLeft(void);
		bool turnSlightRight(void);

	private:
		DriveTrain& robotDrive;
};


#endif
