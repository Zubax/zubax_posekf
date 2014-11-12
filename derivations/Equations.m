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

printMatrixByName["x"]

(* Time update: f(x, dt) *)
f = x;
f[[1;;10]] = Join[
 pwi + dt vwi + dt dt rotateVectorByQuaternion[gravity, Conjugate[qwi]],
 vwi + dt rotateVectorByQuaternion[gravity, Conjugate[qwi]],
 quaternionAsColumnVector[qwi ** simplifiedDeltaQuaternionFromAngularRate[w, dt]]];
F = jacobian[f,x];

printMatrixByName /@ {"f", "F"};

(*
 * Measurement update equations: h(x)
 *)
makeMeasurementPrediction[name_, equation_] := {
  Set[Evaluate[Symbol["Global`h" <> ToString[name]]], FullSimplify[equation]],
  Set[Evaluate[Symbol["Global`H" <> ToString[name]]], FullSimplify[jacobian[equation, x]]]}

makeMeasurementPrediction["acc", a + ba + rotateVectorByQuaternion[gravity, qwi]];
makeMeasurementPrediction["gyro", w + bw];
printMatrixByName /@ {"hacc", "Hacc", "hgyro", "Hgyro"};

makeMeasurementPrediction["gnsspos", pwi];
makeMeasurementPrediction["gnssvel", vwi];
printMatrixByName /@ {"hgnsspos", "Hgnsspos", "hgnssvel", "Hgnssvel"};

makeMeasurementPrediction["vispos", rotateVectorByQuaternion[pwi + pvw, qwi ** qvw]];
makeMeasurementPrediction["visatt", quaternionAsColumnVector[qwi ** qvw]];
printMatrixByName /@ {"hvispos", "Hvispos", "hvisatt", "Hvisatt"};

makeMeasurementPrediction["climbrate", vwi[[3]]];
printMatrixByName /@ {"hclimbrate", "Hclimbrate"};


(*
 * Quaternion covariance to Euler covariance
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 *)
eulerFromQuaternionJacobian = jacobian[eulerFromQuaternion[qwi], List @@ qwi];
printMatrixByName["eulerFromQuaternionJacobian"];

(*
 * GNSS velocity conversion
 *)
gnssVelocityLonLatClimbJacobian =
 jacobian[gnssVelocityLonLatClimb[gnssTrack, gnssSpeed, gnssClimb], {gnssTrack, gnssSpeed, gnssClimb}];
printMatrixByName["gnssVelocityLonLatClimbJacobian"];



