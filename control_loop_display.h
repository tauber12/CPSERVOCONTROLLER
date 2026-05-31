#ifndef CONTROL_LOOP_DISPLAY_H
#define CONTROL_LOOP_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "control.h"

void ControlLoopDisplay_Init(void);
void ControlLoopDisplay_Draw(void);
void ControlLoopDisplay_DrawStatic(void);
void ControlLoopDisplay_DrawForMode(Controller_State mode);
void ControlLoopDisplay_DrawDiagramOnlyForMode(Controller_State mode);
void ControlLoopDisplay_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_DISPLAY_H */
