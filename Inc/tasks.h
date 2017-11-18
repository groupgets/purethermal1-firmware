#ifndef _TASKS_H_
#define _TASKS_H_

#include "pt.h"
#include "usbd_uvc.h"
#include "LEPTON_Types.h"
#include "LEPTON_ErrorCodes.h"

// Long running tasks
PT_THREAD( lepton_task(struct pt *pt));
PT_THREAD( uart_task(struct pt *pt));
PT_THREAD( usb_task(struct pt *pt));
PT_THREAD( button_task(struct pt *pt));
PT_THREAD( lepton_attribute_xfer_task(struct pt *pt));

// Temporary tasks
PT_THREAD( uart_lepton_send(struct pt *pt,char * buffer));
PT_THREAD( LEP_I2C_GetAttribute_PT(struct pt *pt, LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                   LEP_COMMAND_ID commandID,
                                   LEP_ATTRIBUTE_T_PTR attributePtr,
                                   LEP_UINT16 attributeWordLength,
                                   LEP_RESULT *return_code));

PT_THREAD( LEP_I2C_RunCommand_PT(struct pt *pt,
                                 LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                 LEP_COMMAND_ID commandID,
                                 LEP_RESULT *return_code));

PT_THREAD( LEP_I2C_SetAttribute_PT(struct pt *pt,
                                   LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                   LEP_COMMAND_ID commandID,
                                   LEP_ATTRIBUTE_T_PTR attributePtr,
                                   LEP_UINT16 attributeWordLength,
                                   LEP_RESULT *return_code));
// Synchronization helpers
lepton_buffer* dequeue_lepton_buffer(void);
uint32_t get_lepton_buffer(lepton_buffer **buffer);

//Other
void change_overlay_mode(void);
void rgb2yuv(const rgb_t val, uint8_t *y, uint8_t *u, uint8_t *v);

typedef enum {
  UVC_REQUEST_TYPE_ATTR_GET,
  UVC_REQUEST_TYPE_ATTR_SET,
} uvc_request_type;

struct uvc_request {
  uvc_request_type type;
  VC_TERMINAL_ID entity_id;
  uint16_t control_id;
  uint16_t length;
  uint8_t *buffer;
};
HAL_StatusTypeDef enqueue_attribute_xfer_task(struct uvc_request req);

#endif
