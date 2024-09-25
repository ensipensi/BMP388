/***********************************************************Private struct***********************************************************/
#include "stdint.h"
/**
 * @brief struct that saves register values
 * 
 */
typedef struct BmpRegCalibData
{
	/*! Temperature coefficients */
	    uint16_t nvm_par_t1;
	    uint16_t nvm_par_t2;
	    int8_t nvm_par_t3;
	    /*! Pressure coefficients */
	    int16_t nvm_par_p1;
	    int16_t nvm_par_p2;
	    int8_t nvm_par_p3;
	    int8_t nvm_par_p4;
	    uint16_t nvm_par_p5;
	    uint16_t nvm_par_p6;
	    int8_t nvm_par_p7;
	    int8_t nvm_par_p8;
	    int16_t nvm_par_p9;
	    int8_t nvm_par_p10;
	    int8_t nvm_par_p11;

}BmpRegCalibData;


