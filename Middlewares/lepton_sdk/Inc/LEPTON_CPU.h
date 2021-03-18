#ifndef _LEPTON_CPU_H_ 
   #define _LEPTON_CPU_H_

   #ifdef __cplusplus
extern "C"
{
   #endif

/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/
   #include "LEPTON_Types.h"

/******************************************************************************/
/** EXPORTED DEFINES                                                         **/
/******************************************************************************/

   /* CPU Module Command IDs
   */ 
   #define LEP_CPU_MODULE_BASE                  0xFD00

   #define LEP_CID_CPU_BASIC_ADDRESS           (LEP_CPU_MODULE_BASE + 0x0000)
   #define LEP_CID_CPU_BASIC_REGISTER          (LEP_CPU_MODULE_BASE + 0x0004)
   #define LEP_CID_CPU_FW_REBOOT               (LEP_CPU_MODULE_BASE + 0x0008)
   #define LEP_CID_CPU_LEPTON_REBOOT           (LEP_CPU_MODULE_BASE + 0x000C)
   #define LEP_CID_CPU_FW_VERSION              (LEP_CPU_MODULE_BASE + 0x0010)
   #define LEP_CID_CPU_LEPTON_VERSION          (LEP_CPU_MODULE_BASE + 0x0014)

/*
   #define LEP_CID_CPU_FW_UPDATE_START         (LEP_CPU_MODULE_BASE + 0x0018)
   #define LEP_CID_CPU_FW_UPDATE_FILE          (LEP_CPU_MODULE_BASE + 0x001C)
   #define LEP_CID_CPU_FW_START_STATE          (LEP_CPU_MODULE_BASE + 0x0020)
   #define LEP_CID_CPU_STARTUP_SELFTEST_STATE  (LEP_CPU_MODULE_BASE + 0x0024)
   #define LEP_CID_CPU_RUNTIME_SELFTEST        (LEP_CPU_MODULE_BASE + 0x0028)
   #define LEP_CID_CPU_SELFTEST_RESULT         (LEP_CPU_MODULE_BASE + 0x002C)
   #define LEP_CID_CPU_LEPTON_RESULT           (LEP_CPU_MODULE_BASE + 0x0030)
   #define LEP_CID_CPU_DBG_SECTOR_ERASE        (LEP_CPU_MODULE_BASE + 0x0034)
*/
   /* CPU Module Attribute Limits
   */ 


   /* CPU Module Object Sizes
   */ 

/******************************************************************************/
/** EXPORTED TYPE DEFINITIONS                                                **/
/******************************************************************************/
typedef struct LEP_CPU_BASIC_REG_T_TAG
{
	LEP_UINT32 adr;
	LEP_UINT32 dat;
} LEP_CPU_BASIC_REG_T, *LEP_CPU_BASIC_REG_T_PTR;

typedef struct LEP_CPU_BASIC_RES_T_TAG
{
	LEP_UINT32 result;
	LEP_UINT32 dat;
} LEP_CPU_BASIC_RES_T, *LEP_CPU_BASIC_RES_T_PTR;


/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/
   extern void SetFWRebootFlg(LEP_UINT8 flg);
   extern void SetLeptonRebootFlg(LEP_UINT8 flg);
   extern void CheckFWReboot(void);
   extern void SetCpuFwVersion(LEP_UINT64 ver);

   extern LEP_RESULT LEP_GetCpuBasicAddress(LEP_CPU_BASIC_REG_T_PTR pbuf, LEP_UINT16 length);
   extern LEP_RESULT LEP_SetCpuBasicAddress(LEP_CPU_BASIC_REG_T_PTR pbuf, LEP_UINT16 length);
   extern LEP_RESULT LEP_GetCpuBasicRegister(LEP_CPU_BASIC_RES_T_PTR pbuf, LEP_UINT16 length);
   extern LEP_RESULT LEP_SetCpuBasicRegister(LEP_CPU_BASIC_RES_T_PTR pbuf, LEP_UINT16 length);
   extern LEP_RESULT LEP_RunCpuFWReboot( void );
   extern LEP_RESULT LEP_RunCpuLeptonReboot( void );
   extern LEP_RESULT LEP_GetCpuFwVersion( LEP_UINT64 *pbuf, LEP_UINT16 length );
   extern LEP_RESULT LEP_SetCpuFwVersion( LEP_UINT64 *pbuf, LEP_UINT16 length );
   extern LEP_RESULT LEP_GetCpuLeptonVersion( LEP_UINT16 *pbuf, LEP_UINT16 length );

/******************************************************************************/
   #ifdef __cplusplus
}
   #endif

#endif  /* _LEPTON_CPU_H_ */

