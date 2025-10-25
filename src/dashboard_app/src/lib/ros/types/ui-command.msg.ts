export enum UIMode {
  MODE_AUTO = 0,
  MODE_STEP = 1,
  MODE_PAUSED = 2,
  MODE_RESET = 3,
}

export interface UICommandMsg {
  mode: UIMode;
  replan: boolean;
}
