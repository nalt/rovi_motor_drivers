#include <sys/time.h>
#include "tmclLib.h"
#include <stdio.h>
#include <fcntl.h>  
#include <termios.h>
#include <errno.h>  


static char serial=1;

void openPort(void)
{
	
	struct termios SerialPortSettings;

    	
	serial= open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	
	tcgetattr(serial, &SerialPortSettings);

	cfsetispeed(&SerialPortSettings,B9600);	
	cfsetospeed(&SerialPortSettings,B9600);

	SerialPortSettings.c_cflag &= ~PARENB;
	SerialPortSettings.c_cflag &= ~CSTOPB;
	SerialPortSettings.c_cflag &= ~CSIZE;	
	SerialPortSettings.c_cflag &= ~CS8;

	SerialPortSettings.c_cflag &= ~CRTSCTS;
	SerialPortSettings.c_cflag |= CREAD | CLOCAL ;
	
	SerialPortSettings.c_lflag = 0;
	SerialPortSettings.c_cc[VMIN] = 9;  
	SerialPortSettings.c_cc[VTIME] = 0; 
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
	tcsetattr(serial,TCSANOW,&SerialPortSettings);
    
	

}

int SendReceiveData (int value, char command, char typ, char readtyp)
{
	unsigned char writeData[9];
	writeData[0]=1;
	writeData[1]=command;
	writeData[2]=typ;
	writeData[3]=0;
	writeData[4]=value >> 24;
	writeData[5]=value >> 16;
	writeData[6]=value >> 8;
	writeData[7]=value & 0xff;
	writeData[8]=writeData[0]+writeData[1]+writeData[2]+writeData[3]+writeData[4]+writeData[5]+writeData[6]+writeData[7];
	
	write(serial,writeData,sizeof(writeData));

	unsigned char readData[9];
	int returnData;
	read(serial,&readData,9);
	
	if (readtyp==0){
		returnData=readData[2];
	}
	if (readtyp==10) {
		returnData=readData[4]<<24 | readData[5]<<16 | readData[6]<<8 | readData[7];
	}

	return returnData;

}

void closePort(void)
{
    close(serial);
	serial=-1;
}



int setMaxCurrent(int current)
{
	int status;
	status=SendReceiveData (current, 5, 6, 0);
	if (status==100)
	{
		status=SendReceiveData (0, 7, 6, 0);
	}	
	return status;
}


int getMaxCurrent()
{
	int maxCurrent;
	maxCurrent=SendReceiveData (0, 6, 6, 10);							
	return maxCurrent;
}


int setStartCurrent(int current)
{
	int status;
	status=SendReceiveData (current, 5, 177, 0);
	if (status==100)
	{
		status=SendReceiveData (0, 7, 177, 0);
	}
	return status;
}


int getStartCurrent()
{
	int startCurrent;
	startCurrent=SendReceiveData (0, 6, 177, 10);							
	return startCurrent;
}


int setCommutationMethod(int method)
{
	int status;
	status=SendReceiveData (method, 5, 159, 0);
	if (status==100)
	{
		status=SendReceiveData (0, 7, 159, 0);
	}
	return status;
}


int getCommutationMethod()
{
	int commutationMethod;
	commutationMethod=SendReceiveData (0, 6, 159, 10);							
	return commutationMethod;
}


int setMotorPoleNum(int poles)
{
	int status;
	status=SendReceiveData (poles, 5, 253, 0);
	if (status==100)
	{
		status=SendReceiveData (0, 7, 253, 0);
	}
	return status;
}


int getMotorPoleNum()
{
	int poleNum;
	openPort();
	poleNum=SendReceiveData (0, 6, 253, 10);							
	return poleNum;
}


int setMaxVelocity (int velocity)
{
	int status;
	status=SendReceiveData (velocity, 5, 4, 0);
	if (status==100)
	{
		status=SendReceiveData (0, 7, 4, 0);
	}
	return status;		
}


int getMaxVelocity()
{
	int velocity;
	velocity=SendReceiveData (0, 6, 4, 10);							
	return velocity;
}


int setAcceleration (int acceleration)
{
	int status;
	status=SendReceiveData (acceleration, 5, 11, 0);
	if (status==100)
	{
		status=SendReceiveData (0, 7, 11, 0);
	}
	return status;			
}


int getAcceleration()
{
	int acceleration;
	acceleration=SendReceiveData (0, 6, 11, 10);							
	return acceleration;	
}


int stopMotor()
{
	int status;
	status=SendReceiveData (0, 3, 0, 0);	
	return status;
}


int rotateRight(int velocity)
{
	int status;
	status=SendReceiveData (velocity, 1, 0, 0);
	return status;
}


int rotateLeft(int velocity)
{
	int status;
	status=SendReceiveData ( velocity, 2, 0, 0);	
	return status;
}


int moveToPosition(int position)
{
	int status;
	int positionNew;
	
    setActualPos(0);
	positionNew=6.3*position;
	status=SendReceiveData (positionNew, 4, 0, 0);	
	return status;
}


int setTargetPos(int position)
{
	int status;
	int positionNew;
	
    setActualPos(0);
	positionNew=6.3*position;
	status=SendReceiveData(positionNew, 5, 0, 0);						
	return status;
}


int setActualPos(int position)
{
	int status;
	int positionNew;
	
	positionNew=6.3*position;
	status=SendReceiveData(positionNew, 5, 1, 0);						
	return status;	
}


int setTargetVelocity(int velocity)
{
	int status;
	status=SendReceiveData(velocity, 5, 2, 0);						
	return status;
}


int setTargetMotorCurrent(int current)
{
	int status;
	status=SendReceiveData(current, 5, 155, 0);						
	return status;
}


int getTargetPosition()
{
	int position;
	position=SendReceiveData (0, 6, 0, 10);							
	return position;
}


double getActualPosition()
{
	int position;
	double position_Grad;
	position=SendReceiveData ( 0, 6, 1, 10);	
	position_Grad=(position/2268.0)*360.0;					
	return position_Grad;
}


int getTargetVelocity()
{
	int velocity;
	velocity=SendReceiveData (0, 6, 2, 10);								
	return velocity;
}


int getActualVelocity()
{
	int velocity;
	velocity=SendReceiveData (0, 6, 3, 10);								
	return velocity;
}


int getActualMotorCurrent()
{
	int current;
	current=SendReceiveData (0, 6, 150, 10);							
	return current;
}


int getTargetMotorCurrent()
{
	int current;
	current=SendReceiveData (0, 6, 155, 10);							
	return current;	
}


int setPIDdelayPosVel(int delay)
{
	int status;
	status=SendReceiveData (delay, 5, 133, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 133, 0);							
	}	
	return status;
}


int getPIDdelayPosVel()
{
	int delay;
	delay=SendReceiveData (0, 6, 133, 10);							
	return delay;	
	
}


int setPIDdelayCurrent(int delay)
{
	int status;
	status=SendReceiveData (delay, 5, 134, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 134, 0);							
	}
	return status;
}


int getPIDdelayCurrent()
{
	int delay;
	delay=SendReceiveData (0, 6, 134, 10);							
	return delay;	
	
}


int setPIDParamPcurrent(int p)
{
	int status;
	status=SendReceiveData (p, 5, 172, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 172, 0);							
	}
	return status;		
}


int getPIDParamPcurrent()
{
	int P;
	P=SendReceiveData (0, 6, 172, 10);							
	return P;	
	
}


int setParamIcurrent(int I)
{
	int status;
	status=SendReceiveData (I, 5, 173, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 173, 0);							
	}
	return status;		
}


int getPIDParamIcurrent()
{
	int I;
	I=SendReceiveData (0, 6, 173, 10);								
	return I;	
	
}


int setParamPposition(int p)
{
	int status;
	status=SendReceiveData (p, 5, 230, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 230, 0);							
	}
	return status;		
}

int getPIDParamPposition()
{
	int P;
	P=SendReceiveData (0, 6, 230, 10);							
	return P;	
	
}


int setParamPvelocity(int p)
{
	int status;
	status=SendReceiveData (p, 5, 234, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 234, 0);							
	}
	return status;		
}

int getPIDParamPvelocity()
{
	int P;
	P=SendReceiveData (0, 6, 234, 10);							
	return P;	
	
}


int setParamIvelocity(int I)
{
	int status;
	status=SendReceiveData (I, 5, 235, 0);							
	if (status==100)
	{
		status=SendReceiveData (0, 7, 235, 0);							
	}
	return status;		
}


int getPIDParamIvelocity()
{
	int I;
	I=SendReceiveData (0, 6, 235, 10);							
	return I;	
	
}


int getHallAngle()
{
	int Angle;
	Angle=SendReceiveData (0, 6, 210, 10);							
	return Angle;
}


int velocitySensorless(int angle, int reset)
{
	double velocity;
    int targetVelocity;
	static int lastAngle;
	
	static int velocityArray[100];	
	static int velocitySlowArray[10];
	static int velocityVerySlowArray[10];
	static int velocityAverage;	
	int velocitySum;	
	
    static struct timeval last;
    struct timeval now;
	unsigned long timeNow;
	static unsigned long timeLast;
	unsigned long timeDif;
		
	static int a=0;
	static int i=0;
	int s=0;
	int x=0;

	static int aSlow=0;
	static int iSlow=0;
	static int aVerySlow=0;
	static int iVerySlow=0;
	
    
    targetVelocity=getTargetVelocity();
	usleep(100);
    
	if (reset==1)
	{
		lastAngle=0;
        velocityAverage=0;
		timeLast=0;
		a=0;
		i=0;
		aSlow=0;
		iSlow=0;
		for(i=0;i<100;i++)
		{
			velocityArray[i]=0;	
		}
        
        for(i=0;i<10;i++)
        {
            velocitySlowArray[i]=0;
            velocityVerySlowArray[i]=0;
        }

	}	
	else if (((angle==27308) || (angle==16384) || (angle==5461) || (angle==-5461) || (angle==-16383) || (angle==-27305))  && targetVelocity>=800)
	{
            aSlow=0;
			iSlow=0;
			aVerySlow=0;
			iVerySlow=0;
			if (lastAngle!=angle)
			{
				
				gettimeofday(&now,NULL);
				timeNow = (1000000 * now.tv_sec + now.tv_usec);
				timeLast = (1000000 * last.tv_sec + last.tv_usec);
				timeDif=timeNow-timeLast;				

				velocity=(60/12)/(timeDif*0.000001);
				
				if (a==0)
				{
					x=0;
					velocitySum=0;
					velocityArray[i]=velocity;	
					
					for (x=0;x<=i;x++)
					{
						velocitySum=velocityArray[x]+velocitySum;
					}					
					velocityAverage=velocitySum/(i+1);		
					i=i+1;
					if (i>98) 
					{
						a=-1;
											
					}

				}
				else
				{
					
					velocityArray[i]=velocity;
					s=0;
					velocitySum=0;
					for (s=0;s<100;s++)
					{
						velocitySum=velocitySum+velocityArray[s];
					}					
					velocityAverage=0.01*velocitySum;
					i=i+1;
					if (i>99)
					{
						i=0;			
					}


				}		

				lastAngle=angle;
				gettimeofday(&last,NULL);
	
			}	
	
	
	}
	else if (((angle==27308) || (angle==16384) || (angle==5461) || (angle==-5461) || (angle==-16383) || (angle==-27305))  && targetVelocity>=300)
	{
		a=0;
		i=0;
		aVerySlow=0;
		iVerySlow=0;
		if (lastAngle!=angle)
		{
				
				gettimeofday(&now,NULL);
				timeNow = (1000000 * now.tv_sec + now.tv_usec);
				timeLast = (1000000 * last.tv_sec + last.tv_usec);
				timeDif=timeNow-timeLast;				

				velocity=(60/12)/(timeDif*0.000001);
				
				if (aSlow==0)
				{
					x=0;
					velocitySum=0;
					velocitySlowArray[iSlow]=velocity;	
					
					for (x=0;x<=iSlow;x++)
					{
						velocitySum=velocitySlowArray[x]+velocitySum;
					}					
					velocityAverage=velocitySum/(iSlow+1);		
					iSlow=iSlow+1;
					if (iSlow>8) 
					{
						aSlow=-1;
					}
				}
				else
				{
					velocitySlowArray[iSlow]=velocity;
					s=0;
					velocitySum=0;
					for (s=0;s<10;s++)
					{
						velocitySum=velocitySum+velocitySlowArray[s];
					}					
					velocityAverage=0.1*velocitySum;
					iSlow=iSlow+1;
					if (iSlow>9)
					{
						iSlow=0;			
					}
				}		

				lastAngle=angle;
				gettimeofday(&last,NULL);
	
		}	

	}
	else if (((angle==27308) || (angle==16384) || (angle==5461) || (angle==-5461) || (angle==-16383) || (angle==-27305))  && targetVelocity<300)
	{
		a=0;
		i=0;
		aSlow=0;
		iSlow=0;

		gettimeofday(&now,NULL);
		timeNow = (1000000 * now.tv_sec + now.tv_usec);
		timeLast = (1000000 * last.tv_sec + last.tv_usec);
		timeDif=(timeNow-timeLast)*0.000001;
		if (timeDif>1)
		{
			velocityAverage=0;
		}				
	
		if (lastAngle!=angle)
		{
				
				gettimeofday(&now,NULL);
				timeNow = (1000000 * now.tv_sec + now.tv_usec);
				timeLast = (1000000 * last.tv_sec + last.tv_usec);
				timeDif=timeNow-timeLast;				

				velocity=(60/12)/(timeDif*0.000001);
				
				if (aVerySlow==0)
				{
					x=0;
					velocitySum=0;
					velocityVerySlowArray[iVerySlow]=velocity;	
					
					for (x=0;x<=iVerySlow;x++)
					{
						velocitySum=velocityVerySlowArray[x]+velocitySum;
					}					
					velocityAverage=velocitySum/(iVerySlow+1);		
					iVerySlow=iVerySlow+1;
					if (iVerySlow>8) 
					{
						aVerySlow=-1;
					}
				}
				else
				{
					velocityVerySlowArray[iVerySlow]=velocity;
					s=0;
					velocitySum=0;
					for (s=0;s<5;s++)
					{
						velocitySum=velocitySum+velocityVerySlowArray[s];
					}					
					velocityAverage=velocitySum/5;
					iVerySlow=iVerySlow+1;
					if (iVerySlow>4)
					{
						iVerySlow=0;			
					}
				}		

				lastAngle=angle;
				gettimeofday(&last,NULL);
	
		}	

	
	}


	return velocityAverage;

}



double PositionSensorless (int setPosition,int angle)
{
	static double position=2.0; 
	double positionGrad;
	static int oldAngle;
	int newAngle;
	int difference;
	
    
    newAngle=angle;
		
	if (setPosition==1)
	{
		position=2;	
	}	
	else if ((oldAngle!=newAngle))
	{
		difference=oldAngle-newAngle;	
		if ((difference==54613) || (difference==-54613))
		{
			if(difference<0)
			{								
				position=position+1;		
				oldAngle=newAngle;				
			}
			else if (difference>0)
			{					
				position=position-1;
				oldAngle=newAngle;			
			}
		}
		else if (difference>0)
		{
			position=position+1;
			oldAngle=newAngle;	
		}
		else if (difference<0)
		{
			position=position-1;
			oldAngle=newAngle;			
		}
	}
	
	positionGrad=position*(360.0/2268.0);
	return positionGrad;

}


int controlFunction(int simTime, double P, double D, double N, double targetPosition)
{
	double diff;
	int angle;	
	double position_Grad;
	int positionInt;
	double e;
	double e_1=0;
	double e_2=0;
	double u;
	double u_1=0;
	double u_2=0;;
	double Ts=0;
	double umax=4000.0;
	double umin=-500;	
	struct timeval last;
	struct timeval now;
	unsigned long timeNow;
	unsigned long timeLast;
	unsigned long timeDif;
	double	u1,u2,u3,u4,u5;
	int status;
	time_t endwait;
    
    
	endwait = time (NULL) + simTime ;
	FILE *fp;
	fp = fopen("ActualPosition.txt", "w+");
	if (fp==NULL)
	{
	perror("Fehler! ");
	}
	
	position_Grad=PositionSensorless(1,angle);
	
    
    do
	{
		gettimeofday(&last,NULL);
        usleep(100);
        
		angle=getHallAngle();
		
        position_Grad=PositionSensorless(0,angle);
		positionInt=position_Grad;

		

		diff=position_Grad-positionInt;
		if (diff>0.5)
		{
			positionInt=positionInt+1;
		}
		

		e=targetPosition-positionInt;
	

        u1=(-(((N*Ts)-1)*u_1));
		u3=((P+(D*N))*e);
		u4=(e_1*(P*((N*Ts)-1)-(D*N)));
		
		u=u1+u3+u4;

		if (u>umax)
		{
			u=umax;
		}
		if (u<umin) 
		{
			u=umin;
		}
		fprintf(fp, "%i \n", positionInt);
		usleep(100);
		status=setTargetVelocity(u);
	
		u_2=u_1;	
		u_1=u;
		e_2=e_1;		
		e_1=e;
		
		gettimeofday(&now,NULL);
		timeNow = (1000000 * now.tv_sec + now.tv_usec);
		timeLast = (1000000 * last.tv_sec + last.tv_usec);
		timeDif=timeNow-timeLast;	
		Ts=timeDif*0.000001;
		
		
	} while (time (NULL) < endwait);
	
	fclose(fp);
	setTargetVelocity(0);
    return positionInt;

}



