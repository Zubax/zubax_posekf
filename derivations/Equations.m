(* ::Package:: *)

ClearAll["Global`*"];
ClearAll["X`*"];
ClearAll["H`*"];
$ContextPath = Prepend[$ContextPath, "X`"];
$ContextPath = Prepend[$ContextPath, "H`"];
$ContextPath = $ContextPath // DeleteDuplicates
<<NavFunctions`;
<<Quaternions`;
<<CPPCodeGeneration`;


StdGravity = 9.80665;
gravity = {{0}, {0}, {StdGravity}};

(*
 * State vector
 *)
x = Join[
 defineSymbolicColumnVectorXYZ["X`pwi"], (* 1, position *)
 defineSymbolicColumnVectorXYZ["X`vwi"], (* 4, velocity *)
 quaternionAsColumnVector[defineSymbolicQuaternion["X`qwi"]], (* 7, world --> IMU rotation *)
 defineSymbolicColumnVectorXYZ["X`a"], (* 11, acceleration *)
 defineSymbolicColumnVectorXYZ["X`w"], (* 14, angular velocity *)
 defineSymbolicColumnVectorXYZ["X`ba"], (* 17, accelerometer bias *)
 defineSymbolicColumnVectorXYZ["X`bw"], (* 20, rate gyro bias *)
 defineSymbolicColumnVectorXYZ["X`pvw"],(* 23, visual --> world translation *)
 quaternionAsColumnVector[defineSymbolicQuaternion["X`qvw"]]]; (* 26, visual --> world rotation *)

printMatrixByName["x"]

(*
 * Time update: f(x, dt)
 *)
f = x;
f[[1;;10]] = Join[
 pwi + dt vwi + dt dt rotateVectorByQuaternion[gravity, Conjugate[qwi]],
 vwi + dt rotateVectorByQuaternion[gravity, Conjugate[qwi]],
 quaternionAsColumnVector[qwi ** simplifiedDeltaQuaternionFromAngularRate[w, dt]]];
F = jacobian[f,x];
f[[7;;10]] = quaternionAsColumnVector[qwi ** deltaQuaternionFromAngularRate[w, dt]];

printMatrixByName /@ {"f", "F"};

(*
 * State vector normalization
 *)
normx = x;
normx[[7;;10]] = quaternionAsColumnVector[normalizeQuaternion[qwi]];
normx[[26;;29]] = quaternionAsColumnVector[normalizeQuaternion[qvw]];

printMatrixByName @ "normx";

(*
 * Min Q, initial P
 *)
Qmindiag = ConstantArray[0,Length[x]];
Qmindiag[[1;;10]] = ConstantArray[10^-6,10];
Qmindiag[[11;;16]] = ConstantArray[10,6];

Pinitdiag = ConstantArray[10^-6,Length[x]];
Pinitdiag[[1;;10]] = ConstantArray[10^3,10];
Pinitdiag[[11;;16]] = ConstantArray[0.1,6];

printMatrixByName /@ {"Qmindiag", "Pinitdiag"};

(*
 * Measurement update equations: h(x)
 *)
makeMeasurementPrediction[name_, equation_] := {
  Set[Evaluate[Symbol["H`h" <> ToString[name]]], equation],
  Set[Evaluate[Symbol["H`H" <> ToString[name]]], jacobian[equation, x]]}

makeMeasurementPrediction["acc", a + ba + rotateVectorByQuaternion[gravity, qwi]];
makeMeasurementPrediction["gyro", w + bw];

makeMeasurementPrediction["gnsspos", pwi];
makeMeasurementPrediction["gnssvel", vwi];

makeMeasurementPrediction["vispos", rotateVectorByQuaternion[pwi + pvw, qwi ** qvw]];
makeMeasurementPrediction["visatt", qwi ** qvw];

makeMeasurementPrediction["climbrate", {vwi[[3]]}];

printMatrixByName /@ Names["H`*"];


(*
 * Quaternion covariance to Euler covariance
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 *)
eulerFromQuaternionJacobian = jacobian[eulerFromQuaternion[qwi], List @@ qwi];
quaternionFromEulerJacobian = jacobian[quaternionFromEuler[roll, pitch, yaw], {roll, pitch, yaw}];
printMatrixByName["eulerFromQuaternionJacobian"];
printMatrixByName["quaternionFromEulerJacobian"];

(*
 * GNSS velocity conversion
 *)
gnssVelocityLonLatClimbJacobian =
 jacobian[gnssVelocityLonLatClimb[gnssTrack, gnssSpeed, gnssClimb], {gnssTrack, gnssSpeed, gnssClimb}];
printMatrixByName["gnssVelocityLonLatClimbJacobian"];



srcdir = FileNameJoin[{NotebookDirectory[], "..", "src"}];
expandTemplateFiles[srcdir, {"*.cpp", "*.cc", "*.hpp", "*.h"}]



