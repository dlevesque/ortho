
/***** VHT library imports. *****/
#include <vhtCyberGrasp.h>


class CyberForce : public vhtCyberGrasp
{
public:
    CyberForce(void){
		vhtCyberGrasp();
	}

	CyberForce(vhtIOConn *connDict, bool doConnect= true)
		: vhtCyberGrasp(connDict, doConnect)
	{
		
	}

    CyberForce(vhtIOConn *connDict, vhtIOConn *gloveDict, bool doConnect= true)
		: vhtCyberGrasp(connDict, gloveDict, doConnect)
	{
		
	}
 
//	~CyberForce(void){};

     virtual void setForce( double force[8] );
};


