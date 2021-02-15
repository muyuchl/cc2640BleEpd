#ifndef TASK_EPD_H
#define TASK_EPD_H





// -----------------------------------------------------------------------------
//! \brief      API for application task to parse received epd request
//!
//! \param[in]  pMsg    Pointer to "unframed" message buffer.
//! \param[in]  length  Length of buffer
//!
//! \return     void
// ----------------------------------------------------------------------------- 
extern void EPDTask_parseCommand(uint8_t *pMsg, uint8_t length);


// used to call when epd processed request, and response is ready
typedef void (*EpdResponseCallback)(uint8_t event, uint8_t *buf, uint8_t len);
void EPDTask_RegisterResponseCallback(EpdResponseCallback callback);

void TaskEPD_createTask(void);

// called by other task to notify this task with command frame
//void epd_on_rx_cmd(const uint8_t *buf, int len);

#endif