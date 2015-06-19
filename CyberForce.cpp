
#include "CyberForce.h"

#include <iostream>

// Extends vhtCyberGrasp::setForce( double force[5] )
// to enable CyberForce direct force control
void 
CyberForce::setForce( double force[8] )
{
  //std::cout << "setForce" << std::endl;
	char *pos;
	
	pos = raw; // pos points to first byte
	
	for (int j = 0; j < 5; j++ ) {
		
		if(m_forceEffectActive[j])
			pos[0] = 1; // effect active
		else 
			pos[0] = 0;
		
		// nosmalize force for CyberGrasp
		if (force[j] > 1.0){
			m_force[j] = 1.0; 
			VT_PutDouble2(pos+1, 0, 1.0);
		}
		else if (force[j] < 0.0){
			m_force[j] = 0.0;
			VT_PutDouble2(pos+1, 0, 0.0);
		}
		else{
			m_force[j] = force[j]; 
			VT_PutDouble2(pos+1, 0, force[j]);
		}
		
		// pos points to next effect active
		pos = pos + sizeof(double) + 1;
	}
	
	// put CyberForce forces
	for (int j = 5; j < 8; j++){
		VT_PutDouble2(pos, 0, force[j]);
		pos = pos + sizeof(double);
	}
	
	// send them all
	datasz = 5 * sizeof(double) + 5 + (3 * sizeof(double));
	VT_DeviceControl(m_did, DM_SET_DATA, GR_CONTROL_FORCE, raw, &datasz,1024);
	m_currentMode = GR_CONTROL_FORCE;

}