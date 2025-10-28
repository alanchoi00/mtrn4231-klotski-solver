export enum UIMode {
  MODE_IDLE = 0,
  MODE_AUTO = 1,
  MODE_STEP = 2,
  MODE_PAUSED = 3,
  MODE_RESET = 4,
}

export interface UICommandMsg {
  mode: UIMode;
}
