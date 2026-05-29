#ifndef CONTROL_LOOP_DISPLAY_H
#define CONTROL_LOOP_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Landscape 320 x 240 static control-loop screen. */
void ControlLoopDisplay_Init(void);
void ControlLoopDisplay_Draw(void);
void ControlLoopDisplay_DrawStatic(void);

/* Kept for compatibility. Dynamic UI/readout drawing now lives in HMI.c. */
void ControlLoopDisplay_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_DISPLAY_H */
