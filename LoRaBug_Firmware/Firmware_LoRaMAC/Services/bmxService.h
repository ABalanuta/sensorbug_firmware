/******************************************************************************

 @file  bmxService.h

 @brief This file contains the bmx160 Service Interface
        Created on: Jul 11, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_BMXSERVICE_H_
#define SERVICES_BMXSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the BMX Service.
 */
extern void bmxService_createTask(void);

extern void getBmxData(uint8_t *data);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_BMXSERVICE_H_ */
