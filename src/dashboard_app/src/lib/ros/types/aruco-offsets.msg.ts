/**
 * ArUco offsets message type
 * Corresponds to klotski_interfaces/msg/ArucoOffsets
 */
export interface ArucoOffsetsMsg {
  x: number;
  y: number;
  z: number;
}

/**
 * Response from GetArucoOffsets service
 */
export interface GetArucoOffsetsResponse {
  offsets: ArucoOffsetsMsg;
  ok: boolean;
  message: string;
}

/**
 * Response from SetArucoOffsets service
 */
export interface SetArucoOffsetsResponse {
  ok: boolean;
  message: string;
}
