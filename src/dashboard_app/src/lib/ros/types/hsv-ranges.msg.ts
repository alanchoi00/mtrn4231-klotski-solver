// HSV Range message type
export interface HSVRangeMsg {
  name: string;
  h_min: number;
  s_min: number;
  v_min: number;
  h_max: number;
  s_max: number;
  v_max: number;
}

// HSV Ranges message type (collection)
export interface HSVRangesMsg {
  ranges: HSVRangeMsg[];
}

// Service request/response types
export interface GetHSVRangesResponse {
  ranges: HSVRangesMsg;
  ok: boolean;
  message: string;
}

export interface SetHSVRangesRequest {
  ranges: HSVRangesMsg;
}

export interface SetHSVRangesResponse {
  ok: boolean;
  message: string;
}

export interface ExportHSVRangesYamlResponse {
  yaml_content: string;
  ok: boolean;
  message: string;
}

export interface ResetHSVRangesResponse {
  ranges: HSVRangesMsg;
  ok: boolean;
  message: string;
}
