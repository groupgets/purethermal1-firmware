/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/
#include <string.h>
#include "stm32f4xx.h"
#include "LEPTON_SDK.h"
#include "LEPTON_CPU.h"
#include "project_config.h"

/******************************************************************************/
/** LOCAL DEFINES                                                            **/
/******************************************************************************/

/******************************************************************************/
/** LOCAL TYPE DEFINITIONS                                                   **/
/******************************************************************************/

/******************************************************************************/
/** PRIVATE DATA DECLARATIONS                                                **/
/******************************************************************************/
static LEP_UINT8					cpuFWRebootFlg;
static LEP_UINT8					cpuLeptonRebootFlg;
static LEP_UINT64                   cpuFwVersion;
static LEP_CPU_BASIC_REG_T			basic_reg = {0};

/******************************************************************************/
/** PRIVATE FUNCTION DECLARATIONS                                            **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/
extern void board_lepton_init(void);
extern void board_lepton_deinit(void);
extern LEP_UINT16 Get_lepton_type(void);

void SetFWRebootFlg(LEP_UINT8 flg)
{
	cpuFWRebootFlg = flg;
}

void SetLeptonRebootFlg(LEP_UINT8 flg)
{
	cpuLeptonRebootFlg = flg;
}

void CheckFWReboot(void)
{
	if(cpuFWRebootFlg != 0){
	    // wait send response
		HAL_Delay(100);

		HAL_NVIC_SystemReset();
	}
	else if(cpuLeptonRebootFlg != 0){
	    // wait send response
		HAL_Delay(100);

		__disable_irq();
		board_lepton_deinit();
		board_lepton_init();
		__enable_irq();
	}
}

void SetCpuFwVersion(LEP_UINT64 ver)
{
	cpuFwVersion = ver;
}

LEP_RESULT LEP_GetCpuBasicAddress(LEP_CPU_BASIC_REG_T_PTR pbuf, LEP_UINT16 length)
{
  LEP_RESULT  result = LEP_OK;

  memcpy(pbuf, &basic_reg, sizeof(basic_reg));

  return( result );

}

LEP_RESULT LEP_SetCpuBasicAddress(LEP_CPU_BASIC_REG_T_PTR pbuf, LEP_UINT16 length)
{
  LEP_RESULT  result = LEP_OK;

  memcpy(&basic_reg, pbuf, sizeof(basic_reg));

  return( result );
}

LEP_RESULT LEP_GetCpuBasicRegister(LEP_CPU_BASIC_RES_T_PTR pbuf, LEP_UINT16 length)
{
  LEP_RESULT  result = LEP_OK;

  pbuf->dat = *(LEP_UINT32 *)(basic_reg.adr);
  pbuf->result = result;

  return( result );
}

LEP_RESULT LEP_SetCpuBasicRegister(LEP_CPU_BASIC_RES_T_PTR pbuf, LEP_UINT16 length)
{
  LEP_RESULT  result = LEP_OK;

  *(LEP_UINT32 *)(basic_reg.adr) = basic_reg.dat;
  pbuf->result = result;

  return( result );
}

LEP_RESULT LEP_RunCpuFWReboot( void )
{
   LEP_RESULT  result = LEP_OK;

   SetFWRebootFlg(1);

   return( result );
}

LEP_RESULT LEP_RunCpuLeptonReboot( void )
{
   LEP_RESULT  result = LEP_OK;

   SetLeptonRebootFlg(1);

    return( result );
}

LEP_RESULT LEP_GetCpuFwVersion( LEP_UINT64 *pbuf, LEP_UINT16 length )
{
   LEP_RESULT  result = LEP_OK;

   *pbuf = cpuFwVersion;

   return( result );
}

LEP_RESULT LEP_SetCpuFwVersion( LEP_UINT64 *pbuf, LEP_UINT16 length )
{
   LEP_RESULT  result = LEP_OK;

   cpuFwVersion = *pbuf;

   return( result );
}

LEP_RESULT LEP_GetCpuLeptonVersion( LEP_UINT16 *pbuf, LEP_UINT16 length )
{
   LEP_RESULT  result = LEP_OK;

   *pbuf = Get_lepton_type();

   return( result );
}

/******************************************************************************/
/** PRIVATE MODULE FUNCTIONS                                                 **/
/******************************************************************************/

