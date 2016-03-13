#ifndef __USBD_UVC_LEPTON_XU_H
#define __USBD_UVC_LEPTON_XU_H

int8_t VC_LEP_GetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length);
int8_t VC_LEP_SetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length);
int8_t VC_LEP_GetAttributeLen (VC_TERMINAL_ID entity_id, uint16_t offset, uint16_t* pbuf);
int8_t VC_LEP_GetMaxValue (VC_TERMINAL_ID entity_id, uint16_t offset, void* pbuf, uint16_t len);
int8_t VC_LEP_RunCommand (VC_TERMINAL_ID entity_id, uint16_t offset);

#endif