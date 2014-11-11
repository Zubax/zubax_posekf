(* ::Package:: *)

ClearAll["Global`*"];
<<NavFunctions`;
<<Quaternions`;


StdGravity = 9.80665;
gravity = {{0}, {0}, {StdGravity}};

(* State variables *)
pwi = {{pwix}, {pwiy}, {pwiz}}; (* position *)
vwi = {{vwix}, {vwiy}, {vwiz}}; (* velocity *)
qwi = Quaternion[qwiw, qwix, qwiy, qwiz]; (* world --> IMU rotation *)
a   = {{ax}, {ay}, {az}}; (* acceleration *)
w   = {{wx}, {wy}, {wz}}; (* angular velocity *)
ba  = {{bax}, {bay}, {baz}}; (* accelerometer bias *)
bw  = {{bwx}, {bwy}, {bwz}}; (* rate gyro bias *)
pvw = {{pvwx}, {pvwy}, {pvwz}}; (* visual --> world translation *)
qvw = Quaternion[qvww, qvwx, qvwy, qvwz]; (* visual --> world rotation *)

(* State vector *)
x = Join[
 pwi, (* 1 *)
 vwi, (* 4 *)
 quaternionAsColumnVector[qwi], (* 7 *)
 a, (* 11 *)
 w, (* 14 *)
 ba, (* 17 *)
 bw, (* 20 *)
 pvw,(* 23 *)
 quaternionAsColumnVector[qvw]]; (* 26 *)

(* Time update: f(x, dt) *)
f = x;
f[[1;;10]] = Join[
 pwi + dt vwi + dt dt rotateVectorByQuaternion[gravity, Conjugate[qwi]],
 vwi + dt rotateVectorByQuaternion[gravity, Conjugate[qwi]],
 quaternionAsColumnVector[qwi ** simplifiedDeltaQuaternionFromAngularRate[w, dt]]];
F = jacobian[f,x];

(*
 * Measurement update equations: h(x)
 *)


(*
 * Quaternion covariance to Euler covariance
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 *)
eulerFromQuaternionJacobian = jacobian[eulerFromQuaternion[qwi], List@@qwi];

(*
 * GNSS velocity conversion
 *)
gnssVelocityLonLatClimbJacobian =
 jacobian[gnssVelocityLonLatClimb[gnssTrack, gnssSpeed, gnssClimb], {gnssTrack, gnssSpeed, gnssClimb}];


(* Outputs *)
Print["x=", x//MatrixForm]
Print["f=", f//MatrixForm]
Print["F=", F//MatrixForm]

Print["eulerFromQuaternionJacobian=", eulerFromQuaternionJacobian//MatrixForm]
Print["gnssVelocityLonLatClimbJacobian=", gnssVelocityLonLatClimbJacobian//MatrixForm]




